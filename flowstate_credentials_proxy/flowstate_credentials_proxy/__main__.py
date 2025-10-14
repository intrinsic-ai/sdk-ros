import argparse
import asyncio
import sys
from functools import partial

import websockets
import websockets.client
import websockets.exceptions
import websockets.server

from flowstate_credentials_proxy import auth
from flowstate_credentials_proxy.auth import InvalidOrganizationError


async def proxy(
    downstream: websockets.server.WebSocketServerProtocol,
    token_source: auth.TokenSource,
    cluster: str,
    service: str,
):
    """Proxy a single websocket connection to flowstate."""

    token = token_source.get_token()
    uri = f"wss://www.endpoints.{token_source.project}.cloud.goog/onprem/client/{cluster}/api/resourceinstances/{service}"
    headers = {"cookie": f"auth-proxy={token}"}

    try:
        async with websockets.client.connect(uri, extra_headers=headers) as upstream:
            print("Connected to upstream websocket")

            async def forward(src, dst):
                async for msg in src:
                    await dst.send(msg)

            await asyncio.gather(
                forward(downstream, upstream),
                forward(upstream, downstream),
            )
    except websockets.exceptions.InvalidStatusCode as e:
        print(
            f"Upstream connection failed: {e.status_code} {e.headers.get('www-authenticate')}"
        )
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        prog="flowstate_credentials_proxy",
        description="Start a websocket proxy to allow a local zenoh router to connect to flowstate.",
    )
    parser.add_argument(
        "--org",
        required=True,
        help="The organization name in the format organization@project.",
    )
    parser.add_argument("--cluster", required=True, help="Name of the cluster to use.")
    parser.add_argument(
        "--service", required=True, help="Name of the flowstate_ros_bridge instance."
    )
    parser.add_argument("--port", default=7448, help="Port to listen on.")
    args = parser.parse_args()

    try:
        token_source = auth.TokenSource(args.org)
    except FileNotFoundError:
        print(
            f"""Error: Credentials for given organization "{args.org}" not found.
Run 'inctl auth login --org {args.org}' to add it.

Download inctl here https://github.com/intrinsic-ai/sdk/releases""",
            file=sys.stderr,
        )
        return
    except InvalidOrganizationError:
        print("Organization is invalid", file=sys.stderr)
        return
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return

    proxy_handler = partial(
        proxy, token_source=token_source, cluster=args.cluster, service=args.service
    )
    start_server = websockets.server.serve(proxy_handler, "localhost", args.port)

    print(f"Starting zenoh proxy on port {args.port}.")
    asyncio.get_event_loop().run_until_complete(start_server)
    print(
        f"""You may now start rmw_zenohd and connect it to the proxy by running:
```
export ZENOH_CONFIG_OVERRIDE='connect/endpoints=["ws/localhost:{args.port}"];routing/router/peers_failover_brokering=true'
ros2 run rmw_zenoh_cpp rmw_zenohd
```"""
    )

    try:
        asyncio.get_event_loop().run_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
