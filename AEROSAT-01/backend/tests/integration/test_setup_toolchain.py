"""SET-03 (TEST_PLAN section 11.1): the backend toolchain works as configured.

Uses a two-route throwaway FastAPI application defined in this file. No aerosat code is involved.
"""
import importlib.util
import json
import sys
import tomllib
from pathlib import Path

from fastapi import FastAPI, WebSocket
from fastapi.testclient import TestClient

BACKEND = Path(__file__).resolve().parents[2]


def make_app() -> FastAPI:
    app = FastAPI()

    @app.get("/ping")
    def ping() -> dict[str, str]:
        return {"pong": "ok"}

    @app.websocket("/ws")
    async def ws(socket: WebSocket) -> None:
        await socket.accept()
        await socket.send_text(json.dumps({"type": "hello"}))
        await socket.close()

    return app


def test_set_03_python_version_meets_nfr_08() -> None:
    assert sys.version_info >= (3, 11)


def test_set_03_pytest_tiers_are_configured() -> None:
    config = tomllib.loads((BACKEND / "pyproject.toml").read_text(encoding="utf-8"))["tool"]["pytest"]["ini_options"]
    marker_names = {marker.split(":")[0] for marker in config["markers"]}
    assert marker_names == {"slow", "perf"}
    assert "--strict-markers" in config["addopts"]


def test_set_03_coverage_plugin_is_installed() -> None:
    assert importlib.util.find_spec("pytest_cov") is not None


def test_set_03_test_client_uses_httpx2() -> None:
    import starlette.testclient

    assert starlette.testclient.httpx.__name__ == "httpx2"
    assert importlib.util.find_spec("httpx") is None  # the deprecated fallback is not installed


def test_set_03_rest_round_trip_through_the_test_client() -> None:
    with TestClient(make_app()) as client:
        response = client.get("/ping")
    assert response.status_code == 200
    assert response.json() == {"pong": "ok"}


def test_set_03_websocket_round_trip_through_the_test_client() -> None:
    with TestClient(make_app()) as client, client.websocket_connect("/ws") as socket:
        assert json.loads(socket.receive_text()) == {"type": "hello"}


def test_set_03_uvicorn_has_a_websocket_implementation() -> None:
    """uvicorn[standard] supplies `websockets`; without it Uvicorn cannot serve WebSocket endpoints."""
    assert importlib.util.find_spec("uvicorn") is not None
    assert importlib.util.find_spec("websockets") is not None
