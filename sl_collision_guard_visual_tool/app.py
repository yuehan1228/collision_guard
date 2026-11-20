from __future__ import annotations

import json
import mimetypes
import os
import shutil
import subprocess
import tempfile
from dataclasses import dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse


BASE_DIR = Path(__file__).resolve().parent
STATIC_DIR = BASE_DIR / "static"
CONFIG_FILE = Path(
    os.environ.get(
        "CG_DEBUG_TOOL_CONFIG",
        os.environ.get("SERVICE_ADMIN_CONFIG", BASE_DIR / "config" / "services.json"),
    )
).expanduser()
DEFAULT_HOST = os.environ.get(
    "CG_DEBUG_TOOL_HOST", os.environ.get("SERVICE_ADMIN_HOST", "0.0.0.0")
)
DEFAULT_PORT = int(
    os.environ.get("CG_DEBUG_TOOL_PORT", os.environ.get("SERVICE_ADMIN_PORT", "8080"))
)
SYSTEMCTL = shutil.which("systemctl") or "systemctl"
SUDO = shutil.which("sudo") or "sudo"


@dataclass(frozen=True)
class ServiceEntry:
    service_id: str
    display_name: str
    unit: str
    visualization_unit: str | None
    web_visualization_unit: str | None
    status_bridge_unit: str | None
    webviz_lidars: list[str]
    webviz_point_topic_template: str | None
    webviz_protection_topic_template: str | None
    progress_path: str | None
    config_path: str | None
    config_format: str
    restart_after_save: bool


class ApiError(Exception):
    def __init__(self, status: int, message: str):
        super().__init__(message)
        self.status = status
        self.message = message


def load_services() -> dict[str, ServiceEntry]:
    raw = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
    services: dict[str, ServiceEntry] = {}
    for item in raw.get("services", []):
        service = ServiceEntry(
            service_id=item["id"],
            display_name=item.get("display_name", item["id"]),
            unit=item["unit"],
            visualization_unit=item.get("visualization_unit"),
            web_visualization_unit=item.get("web_visualization_unit"),
            status_bridge_unit=item.get("status_bridge_unit"),
            webviz_lidars=list(item.get("webviz_lidars", [])),
            webviz_point_topic_template=item.get("webviz_point_topic_template"),
            webviz_protection_topic_template=item.get(
                "webviz_protection_topic_template"
            ),
            progress_path=item.get("progress_path"),
            config_path=item.get("config_path"),
            config_format=item.get("config_format", "text"),
            restart_after_save=bool(item.get("restart_after_save", False)),
        )
        services[service.service_id] = service
    return services


def get_service(service_id: str) -> ServiceEntry:
    services = load_services()
    service = services.get(service_id)
    if service is None:
        raise ApiError(HTTPStatus.NOT_FOUND, f"Unknown service: {service_id}")
    return service


def run_command(args: list[str], timeout: int = 30) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            args,
            capture_output=True,
            text=True,
            timeout=timeout,
            check=False,
        )
    except FileNotFoundError as exc:
        raise ApiError(
            HTTPStatus.INTERNAL_SERVER_ERROR, f"Command not found: {args[0]}"
        ) from exc
    except subprocess.TimeoutExpired as exc:
        raise ApiError(
            HTTPStatus.GATEWAY_TIMEOUT, f"Command timed out: {' '.join(args)}"
        ) from exc


def systemctl_args(action: str, unit: str | None = None) -> list[str]:
    args = [SYSTEMCTL, "--no-ask-password", action]
    if unit:
        args.append(unit)
    if os.geteuid() != 0:
        return [SUDO, "-n", *args]
    return args


def ensure_success(result: subprocess.CompletedProcess[str], action: str) -> None:
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "unknown error").strip()
        raise ApiError(HTTPStatus.BAD_GATEWAY, f"{action} failed: {detail}")


def read_systemd_show(unit: str) -> dict[str, str]:
    result = run_command(
        [
            SYSTEMCTL,
            "show",
            unit,
            "--property",
            "Id,Description,LoadState,ActiveState,SubState,UnitFileState,MainPID,ExecMainStartTimestamp",
        ]
    )
    ensure_success(result, f"Read status for {unit}")

    data: dict[str, str] = {}
    for line in result.stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        data[key] = value
    return data


def service_status(service: ServiceEntry) -> dict[str, Any]:
    show = read_systemd_show(service.unit)
    return {
        "id": service.service_id,
        "display_name": service.display_name,
        "unit": service.unit,
        "description": show.get("Description", ""),
        "load_state": show.get("LoadState", "unknown"),
        "active_state": show.get("ActiveState", "unknown"),
        "sub_state": show.get("SubState", "unknown"),
        "unit_file_state": show.get("UnitFileState", "unknown"),
        "enabled": show.get("UnitFileState", "").startswith("enabled"),
        "main_pid": show.get("MainPID", "0"),
        "started_at": show.get("ExecMainStartTimestamp", ""),
        "visualization_unit": service.visualization_unit,
        "web_visualization_unit": service.web_visualization_unit,
        "status_bridge_unit": service.status_bridge_unit,
        "webviz_lidars": service.webviz_lidars,
        "webviz_point_topic_template": service.webviz_point_topic_template,
        "webviz_protection_topic_template": service.webviz_protection_topic_template,
        "progress_path": service.progress_path,
        "config_path": service.config_path,
        "config_format": service.config_format,
        "restart_after_save": service.restart_after_save,
    }


def service_progress(service: ServiceEntry) -> dict[str, Any]:
    if not service.progress_path:
        raise ApiError(
            HTTPStatus.BAD_REQUEST, f"{service.service_id} has no progress_path"
        )

    progress_path = Path(service.progress_path)
    if not progress_path.exists():
        return {
            "id": service.service_id,
            "state": "idle",
            "message": "未开始",
            "progress": 0,
            "current_frames": 0,
            "total_frames": 0,
            "completed_lidars": 0,
            "total_lidars": 0,
            "per_lidar": {},
        }

    try:
        payload = json.loads(progress_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise ApiError(
            HTTPStatus.BAD_GATEWAY,
            f"Failed to read progress file: {progress_path}",
        ) from exc

    if not isinstance(payload, dict):
        raise ApiError(
            HTTPStatus.BAD_GATEWAY, f"Invalid progress payload: {progress_path}"
        )

    payload["id"] = service.service_id
    return payload


def write_config_file(target_path: Path, content: str) -> None:
    target_path.parent.mkdir(parents=True, exist_ok=True)
    fd, temp_path = tempfile.mkstemp(
        prefix=f".{target_path.name}.", dir=target_path.parent
    )
    os.close(fd)
    temp_file = Path(temp_path)
    try:
        temp_file.write_text(content, encoding="utf-8")
        if target_path.exists():
            shutil.copymode(target_path, temp_file)
        os.replace(temp_file, target_path)
    finally:
        if temp_file.exists():
            temp_file.unlink()


class ServiceAdminHandler(BaseHTTPRequestHandler):
    server_version = "CollisionGuardVisualDebugger/0.1"

    def do_GET(self) -> None:
        try:
            self.route_get()
        except ApiError as exc:
            self.send_json(exc.status, {"error": exc.message})

    def do_POST(self) -> None:
        try:
            self.route_post()
        except ApiError as exc:
            self.send_json(exc.status, {"error": exc.message})

    def do_PUT(self) -> None:
        try:
            self.route_put()
        except ApiError as exc:
            self.send_json(exc.status, {"error": exc.message})

    def log_message(self, fmt: str, *args: Any) -> None:
        return

    def route_get(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/api/services":
            services = [service_status(service) for service in load_services().values()]
            self.send_json(HTTPStatus.OK, {"services": services})
            return

        if path.startswith("/api/services/"):
            service_id, suffix = self.parse_service_path(path)
            service = get_service(service_id)
            if suffix == "":
                self.send_json(HTTPStatus.OK, service_status(service))
                return
            if suffix == "/config":
                if not service.config_path:
                    raise ApiError(
                        HTTPStatus.BAD_REQUEST, f"{service_id} has no config_path"
                    )
                config_path = Path(service.config_path)
                if not config_path.exists():
                    raise ApiError(
                        HTTPStatus.NOT_FOUND, f"Config file not found: {config_path}"
                    )
                self.send_json(
                    HTTPStatus.OK,
                    {
                        "path": service.config_path,
                        "format": service.config_format,
                        "content": config_path.read_text(encoding="utf-8"),
                    },
                )
                return
            if suffix == "/progress":
                self.send_json(HTTPStatus.OK, service_progress(service))
                return
            if suffix == "/logs":
                query = parse_qs(parsed.query)
                try:
                    requested_lines = int(query.get("lines", ["200"])[0])
                except ValueError as exc:
                    raise ApiError(
                        HTTPStatus.BAD_REQUEST, "lines must be an integer"
                    ) from exc
                lines = min(max(requested_lines, 1), 2000)
                result = run_command(
                    [
                        "journalctl",
                        "-u",
                        service.unit,
                        "-n",
                        str(lines),
                        "--no-pager",
                        "-o",
                        "short-iso",
                    ],
                    timeout=60,
                )
                ensure_success(result, f"Read logs for {service.unit}")
                self.send_json(
                    HTTPStatus.OK,
                    {"unit": service.unit, "lines": lines, "content": result.stdout},
                )
                return

        self.serve_static(path)

    def route_post(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if path.startswith("/api/services/"):
            service_id, suffix = self.parse_service_path(path)
            if suffix != "/action":
                raise ApiError(HTTPStatus.NOT_FOUND, f"Unknown endpoint: {path}")
            service = get_service(service_id)
            body = self.read_json_body()
            action = body.get("action")
            valid_actions = {
                "start",
                "stop",
                "restart",
                "enable",
                "disable",
                "visualize",
                "web_visualize",
                "start_bridge",
            }
            if action not in valid_actions:
                raise ApiError(HTTPStatus.BAD_REQUEST, f"Unsupported action: {action}")
            if action == "visualize":
                if not service.visualization_unit:
                    raise ApiError(
                        HTTPStatus.BAD_REQUEST,
                        f"{service.service_id} has no visualization_unit",
                    )
                result = run_command(systemctl_args("start", service.visualization_unit))
                ensure_success(result, f"start {service.visualization_unit}")
            elif action == "web_visualize":
                if not service.web_visualization_unit:
                    raise ApiError(
                        HTTPStatus.BAD_REQUEST,
                        f"{service.service_id} has no web_visualization_unit",
                    )
                result = run_command(
                    systemctl_args("start", service.web_visualization_unit)
                )
                ensure_success(result, f"start {service.web_visualization_unit}")
            elif action == "start_bridge":
                if not service.status_bridge_unit:
                    raise ApiError(
                        HTTPStatus.BAD_REQUEST,
                        f"{service.service_id} has no status_bridge_unit",
                    )
                result = run_command(systemctl_args("start", service.status_bridge_unit))
                ensure_success(result, f"start {service.status_bridge_unit}")
            else:
                result = run_command(systemctl_args(action, service.unit))
                ensure_success(result, f"{action} {service.unit}")
                if action == "start" and service.status_bridge_unit:
                    bridge_result = run_command(
                        systemctl_args("start", service.status_bridge_unit)
                    )
                    ensure_success(
                        bridge_result, f"start {service.status_bridge_unit}"
                    )
            self.send_json(
                HTTPStatus.OK,
                {"message": f"{action} succeeded", "status": service_status(service)},
            )
            return
        raise ApiError(HTTPStatus.NOT_FOUND, f"Unknown endpoint: {path}")

    def route_put(self) -> None:
        parsed = urlparse(self.path)
        path = parsed.path
        if not path.startswith("/api/services/"):
            raise ApiError(HTTPStatus.NOT_FOUND, f"Unknown endpoint: {path}")
        service_id, suffix = self.parse_service_path(path)
        if suffix != "/config":
            raise ApiError(HTTPStatus.NOT_FOUND, f"Unknown endpoint: {path}")
        service = get_service(service_id)
        if not service.config_path:
            raise ApiError(HTTPStatus.BAD_REQUEST, f"{service_id} has no config_path")
        body = self.read_json_body()
        content = body.get("content")
        if not isinstance(content, str):
            raise ApiError(HTTPStatus.BAD_REQUEST, "content must be a string")
        restart = bool(body.get("restart", service.restart_after_save))
        config_path = Path(service.config_path)
        write_config_file(config_path, content)
        reload_result = run_command(systemctl_args("daemon-reload"))
        ensure_success(reload_result, "systemctl daemon-reload")
        if restart:
            result = run_command(systemctl_args("restart", service.unit))
            ensure_success(result, f"restart {service.unit}")
        self.send_json(
            HTTPStatus.OK,
            {
                "message": "Config saved",
                "restarted": restart,
                "status": service_status(service),
            },
        )

    def parse_service_path(self, path: str) -> tuple[str, str]:
        prefix = "/api/services/"
        remainder = path[len(prefix) :]
        if not remainder:
            raise ApiError(HTTPStatus.NOT_FOUND, f"Unknown endpoint: {path}")
        if "/" in remainder:
            service_id, suffix = remainder.split("/", 1)
            return service_id, f"/{suffix}"
        return remainder, ""

    def read_json_body(self) -> dict[str, Any]:
        content_length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(content_length) if content_length > 0 else b"{}"
        try:
            data = json.loads(raw.decode("utf-8"))
        except json.JSONDecodeError as exc:
            raise ApiError(HTTPStatus.BAD_REQUEST, "Invalid JSON body") from exc
        if not isinstance(data, dict):
            raise ApiError(HTTPStatus.BAD_REQUEST, "JSON body must be an object")
        return data

    def send_json(self, status: int, payload: dict[str, Any]) -> None:
        encoded = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(encoded)))
        self.end_headers()
        self.wfile.write(encoded)

    def serve_static(self, path: str) -> None:
        if path in {"", "/"}:
            file_path = STATIC_DIR / "index.html"
        else:
            relative = path.lstrip("/")
            file_path = (STATIC_DIR / relative).resolve()
            if not str(file_path).startswith(str(STATIC_DIR.resolve())):
                raise ApiError(HTTPStatus.FORBIDDEN, "Invalid path")

        if not file_path.exists() or not file_path.is_file():
            raise ApiError(HTTPStatus.NOT_FOUND, f"File not found: {path}")

        content_type = (
            mimetypes.guess_type(file_path.name)[0] or "application/octet-stream"
        )
        data = file_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def main() -> None:
    if not CONFIG_FILE.exists():
        raise SystemExit(f"Missing configuration file: {CONFIG_FILE}")
    server = ThreadingHTTPServer((DEFAULT_HOST, DEFAULT_PORT), ServiceAdminHandler)
    print(
        "Collision guard visual debugger listening on "
        f"http://{DEFAULT_HOST}:{DEFAULT_PORT}"
    )
    server.serve_forever()


if __name__ == "__main__":
    main()
