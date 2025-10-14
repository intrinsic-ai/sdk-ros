import base64
import json
import pathlib
import re
from datetime import datetime, timedelta, timezone
from os import environ

import aiohttp

INTRINSIC_CONFIG_LOCATION = pathlib.Path(environ["HOME"], ".config", "intrinsic")
ORGANIZATION_PATTERN = re.compile(r"[a-zA-Z][\w-]*@[a-zA-Z][\w-]*")
FLOWSTATE_ADDR = "https://flowstate.intrinsic.ai"


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
    def __init__(self, org: str):
        """
        Create a new instance of TokenSource.

        Raies:
            InvalidOrganizationError: If the organization is invalid.
            JSONDecodeError: If the API key file is malformed.
            KeyError: If the API key file is invalid.
            FileNotFoundError: If the API key file does not exist.
        """
        self.org = org
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
                access_token: str = (await resp.json())["idToken"]
                self.__cached_token = access_token
                return self.__cached_token
