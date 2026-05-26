import base64
import json
import pathlib
import re
from datetime import datetime, timedelta, timezone
from os import environ

import aiohttp

USE_SSO = bool(environ.get("FLOWSTATE_CREDENTIALS_PROXY_USE_SSO", False))

INTRINSIC_CONFIG_LOCATION = pathlib.Path(environ["HOME"], ".config", "intrinsic")
ORGANIZATION_PATTERN = re.compile(r"[a-zA-Z][\w-]*@[a-zA-Z][\w-]*")
if USE_SSO:
    # TODO: sso only works on dev right now.
    FLOWSTATE_ADDR = "https://flowstate-dev.intrinsic.ai"
    FLOWSTATE_ACCOUNTS_ADDR = "https://accounts-dev.intrinsic.ai"
else:
    FLOWSTATE_ADDR = "https://flowstate.intrinsic.ai"
    FLOWSTATE_ACCOUNTS_ADDR = "https://accounts.intrinsic.ai"


class InvalidOrganizationError(ValueError):
    pass


def get_project_from_organization(org: str) -> str:
    """
    Get the project from an organization@project string.

    Raises:
        InvalidOrganizationError: If the organization is invalid.
    """
    if ORGANIZATION_PATTERN.fullmatch(org) is None:
        raise InvalidOrganizationError("invalid organization")
    return org.split("@")[1]


def get_api_key(project: str):
    """
    Get the cached API key for an organization.

    Raies:
        JSONDecodeError: If the API key file is malformed.
        KeyError: If the API key file is invalid.
        FileNotFoundError: If the API key file does not exist.
    """
    user_token = json.load(
        pathlib.Path(
            INTRINSIC_CONFIG_LOCATION, "projects", f"{project}.user-token"
        ).open()
    )
    api_key = user_token["tokens"]["default"]["apiKey"]
    return api_key


class TokenSource:
    def __init__(self, org: str, cluster: str):
        """
        Create a new instance of TokenSource.

        Raies:
            InvalidOrganizationError: If the organization is invalid.
            JSONDecodeError: If the API key file is malformed.
            KeyError: If the API key file is invalid.
            FileNotFoundError: If the API key file does not exist.
        """
        self.org = org
        self.cluster = cluster
        self.project = get_project_from_organization(self.org)
        self.api_key = get_api_key(self.project)
        self.__cached_token: str | None = None

    @staticmethod
    def __token_expiring(token: str) -> bool:
        payload = json.loads(base64.b64decode(token.split(".")[1] + "=="))
        return datetime.now(timezone.utc) > (
            datetime.fromtimestamp(payload["exp"], tz=timezone.utc)
            - timedelta(minutes=5)
        )

    async def get_token(self) -> str:
        """
        Get an access token.

        Raises:
            aiohttp.ClientError: If the request fails.
            KeyError: Cannot find the access token in the response.
        """
        if self.__cached_token is not None and not self.__token_expiring(
            self.__cached_token
        ):
            return self.__cached_token

        async with aiohttp.ClientSession(
            base_url=FLOWSTATE_ADDR,
            headers={"content-type": "application/json"},
            raise_for_status=True,
        ) as session:
            async with session.post(
                f"/api/v1/accountstokens:idtoken",
                json={
                    "apiKey": self.api_key,
                    "do_fan_out": False,
                    "api_key_project_hint": self.project,
                },
            ) as resp:
                token: str | None = (await resp.json())["idToken"]
                if not token:
                    raise KeyError("idToken not found in response")
                self.__cached_token = token

        if USE_SSO:
            async with aiohttp.ClientSession(
                base_url=FLOWSTATE_ACCOUNTS_ADDR,
                cookies={"auth-proxy": self.__cached_token},
                raise_for_status=True,
            ) as session:
                async with session.post(
                    f"/sso/v1alpha/equipment/org/{self.org}/cluster/{self.cluster}/code",
                ) as resp:
                    sso_code = (await resp.json())["code"]

                async with session.post(
                    f"/sso/v1alpha/equipment/signin",
                    json={
                        "code": sso_code,
                    },
                ) as resp:
                    token: str | None = (await resp.json())["signin_token"]
                    if not token:
                        raise KeyError("signin_token not found in response")
                    self.__cached_token = token

        return self.__cached_token
