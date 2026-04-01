import argparse
import asyncio
import sys

import aiohttp
import grpc
from aiohttp import web
from flowstate_credentials_proxy import auth
from flowstate_credentials_proxy.auth import InvalidOrganizationError
from flowstate_credentials_proxy.grpc import create_onprem_channel_async
from intrinsic.assets.proto.id_pb2 import Id
from intrinsic.assets.proto.installed_assets_pb2 import (
    GetInstalledAssetRequest,
    InstalledAsset,
)
from intrinsic.assets.proto.installed_assets_pb2_grpc import InstalledAssetsStub


async def proxy(
    req: web.Request, token_source: auth.TokenSource, cluster: str, service: str
) -> web.WebSocketResponse:
    """Proxy a single websocket connection to flowstate."""

    ws_response = web.WebSocketResponse()
    await ws_response.prepare(req)

    try:
        token = await token_source.get_token()
    except Exception as e:
        print(f"Error getting token: {e}", file=sys.stderr)
        await ws_response.close()
        return ws_response

    uri = f"wss://www.endpoints.{token_source.project}.cloud.goog/onprem/client/{cluster}/api/resourceinstances/{service}"
    headers = {"cookie": f"auth-proxy={token}"}

    try:
        async with aiohttp.ClientSession() as session:
            async with session.ws_connect(uri, headers=headers) as upstream:
                print("Connected to upstream websocket")

                async def forward_to_upstream(
                    d: web.WebSocketResponse, u: aiohttp.ClientWebSocketResponse
                ):
                    async for msg in d:
                        if msg.type == aiohttp.WSMsgType.TEXT:
                            await u.send_str(msg.data)
                        elif msg.type == aiohttp.WSMsgType.BINARY:
                            await u.send_bytes(msg.data)
                        elif msg.type == aiohttp.WSMsgType.CLOSE:
                            await u.close()
                            break
                        else:
                            break

                async def forward_to_downstream(
                    u: aiohttp.ClientWebSocketResponse, d: web.WebSocketResponse
                ):
                    async for msg in u:
                        if msg.type == aiohttp.WSMsgType.TEXT:
                            await d.send_str(msg.data)
                        elif msg.type == aiohttp.WSMsgType.BINARY:
                            await d.send_bytes(msg.data)
                        elif msg.type in (
                            aiohttp.WSMsgType.CLOSED,
                            aiohttp.WSMsgType.ERROR,
                        ):
                            await d.close(code=u.close_code or 1000)
                            break

                await asyncio.gather(
                    forward_to_upstream(ws_response, upstream),
                    forward_to_downstream(upstream, ws_response),
                )
    except aiohttp.ClientResponseError as e:
        if e.status == 404:
            print(
                f"Failed to connect to {e.request_info.url}. Make sure that the cluster and service name is correct.",
                file=sys.stderr,
            )
        else:
            raise e
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
    finally:
        return ws_response


class InitFlowstateRosBridgeError(Exception):
    pass


async def init_flowstate_ros_bridge(token_source: auth.TokenSource, cluster: str):
    #     token = await token_source.get_token()
    #     print(
    #         f"""Successfully authenticated to flowstate.
    # You may now start rmw_zenohd and connect it to the proxy by running:

    # ```
    # ZENOH_CONFIG_OVERRIDE='connect/endpoints=["ws/localhost:{args.port}"];routing/router/peers_failover_brokering=true' ros2 run rmw_zenoh_cpp rmw_zenohd
    # ```"""
    #     )

    # try:
    #     loop.run_until_complete(check_token())
    # except aiohttp.ClientResponseError as e:
    #     if e.status == 401:
    #         print(f"Failed to authenticate to flowstate: {e.message}", file=sys.stderr)
    #         print(
    #             f"Try refreshing your api key `inctl auth login --org={args.org}`",
    #             file=sys.stderr,
    #         )
    #         sys.exit(401)
    #     else:
    #         raise e

    channel = create_onprem_channel_async(token_source, cluster)
    async with channel:
        installed_assets_client = InstalledAssetsStub(channel)
        try:
            installed_asset: InstalledAsset = (
                await installed_assets_client.GetInstalledAsset(
                    GetInstalledAssetRequest(
                        id=Id(package="ai.intrinsic", name="flowstate_ros_bridgea")
                    )
                )
            )
            print(installed_asset)
        except grpc.aio.AioRpcError as e:
            if e.code() == grpc.StatusCode.NOT_FOUND:
                # TODO: ask user if they want to install
                raise InitFlowstateRosBridgeError(
                    "flowstate_ros_bridge is not installed"
                ) from e
            else:
                raise e


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

    try:
        asyncio.run(init_flowstate_ros_bridge(token_source, args.cluster))
    except InitFlowstateRosBridgeError as e:
        print(e)
        exit(1)

    print(f"Starting zenoh proxy on port {args.port}.")

    # app = web.Application()
    # app.add_routes(
    #     [web.get("/", lambda req: proxy(req, token_source, args.cluster, args.service))]
    # )

    # web.run_app(app, port=args.port, handle_signals=True, print=None)


if __name__ == "__main__":
    main()
