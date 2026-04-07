import grpc
from flowstate_credentials_proxy.auth import TokenSource


class AsyncAuthInterceptor(
    grpc.aio.UnaryUnaryClientInterceptor,
    grpc.aio.UnaryStreamClientInterceptor,
    grpc.aio.StreamUnaryClientInterceptor,
    grpc.aio.StreamStreamClientInterceptor,
):
    def __init__(self, token_source: TokenSource, cluster: str):
        self._token_source = token_source
        self._cluster = cluster

    async def intercept_call(
        self, continuation, client_call_details: grpc.aio.ClientCallDetails, request
    ):
        token = await self._token_source.get_token()

        metadata = (
            ("cookie", f"auth-proxy={token}"),
            ("cookie", f"org-id={self._token_source.project}"),
            ("x-server-name", self._cluster),
        )

        if client_call_details.metadata is None:
            client_call_details.metadata = grpc.aio.Metadata()
        for k, v in metadata:
            client_call_details.metadata.add(k, v)

        return await continuation(client_call_details, request)

    async def intercept_unary_unary(self, continuation, client_call_details, request):
        return await self.intercept_call(continuation, client_call_details, request)

    async def intercept_unary_stream(self, continuation, client_call_details, request):
        return await self.intercept_call(continuation, client_call_details, request)

    async def intercept_stream_stream(
        self, continuation, client_call_details, request_iterator
    ):
        return await self.intercept_call(
            continuation, client_call_details, request_iterator
        )

    async def intercept_stream_unary(
        self, continuation, client_call_details, request_iterator
    ):
        return await self.intercept_call(
            continuation, client_call_details, request_iterator
        )


def create_onprem_channel_async(
    token_source: TokenSource, cluster: str
) -> grpc.aio.Channel:
    return grpc.aio.secure_channel(
        f"www.endpoints.{token_source.project}.cloud.goog:443",
        grpc.ssl_channel_credentials(),
        options=[("grpc.max_receive_message_length", -1)],
        interceptors=[AsyncAuthInterceptor(token_source, cluster)],
    )
