import json
import unittest
from unittest.mock import MagicMock, mock_open, patch

import requests

from flowstate_credentials_proxy.auth import (
    InvalidOrganizationError,
    get_access_token,
    get_api_key,
)


class TestGetApiKey(unittest.TestCase):
    @patch("pathlib.Path.open", new_callable=mock_open)
    def test_get_api_key_success(self, mock_open_file: MagicMock) -> None:
        mock_user_token = {
            "name": "test_project",
            "tokens": {"default": {"apiKey": "test-api-key"}},
        }
        mock_open_file.return_value.read.return_value = json.dumps(mock_user_token)
        api_key = get_api_key("user@project")
        self.assertEqual(api_key, "test-api-key")

    def test_get_api_key_invalid_organization(self) -> None:
        with self.assertRaises(InvalidOrganizationError) as e:
            get_api_key("invalid-organization")
        self.assertEqual(str(e.exception), "invalid organization")

    @patch("pathlib.Path.open", new_callable=mock_open)
    def test_get_api_key_invalid_json(self, mock_open_file: MagicMock) -> None:
        mock_open_file.return_value.read.return_value = "invalid json"
        with self.assertRaises(json.JSONDecodeError):
            get_api_key("user@project")

    @patch("pathlib.Path.open", new_callable=mock_open)
    def test_get_api_key_missing_key(self, mock_open_file: MagicMock) -> None:
        mock_open_file.return_value.read.return_value = json.dumps({})
        with self.assertRaises(KeyError):
            get_api_key("user@project")


class TestGetAccessToken(unittest.TestCase):
    @patch("flowstate_credentials_proxy.auth.requests.post")
    def test_get_access_token_success(self, mock_post: MagicMock) -> None:
        mock_response = MagicMock()
        mock_response.json.return_value = {"idToken": "test-access-token"}
        mock_response.raise_for_status.return_value = None
        mock_post.return_value = mock_response

        access_token = get_access_token("test-project", "test-api-key")

        self.assertEqual(access_token, "test-access-token")
        mock_post.assert_called_once_with(
            "https://flowstate.intrinsic.ai/api/v1/accountstokens:idtoken",
            headers={"content-type": "application/json"},
            json={
                "apiKey": "test-api-key",
                "do_fan_out": False,
                "api_key_project_hint": "test-project",
            },
        )

    @patch("flowstate_credentials_proxy.auth.requests.post")
    def test_get_access_token_http_error(self, mock_post: MagicMock) -> None:
        mock_response = MagicMock()
        mock_response.raise_for_status.side_effect = requests.exceptions.HTTPError
        mock_post.return_value = mock_response

        with self.assertRaises(requests.exceptions.HTTPError):
            get_access_token("test-project", "test-api-key")

    @patch("flowstate_credentials_proxy.auth.requests.post")
    def test_get_access_token_missing_key(self, mock_post: MagicMock) -> None:
        mock_response = MagicMock()
        mock_response.json.return_value = {}
        mock_response.raise_for_status.return_value = None
        mock_post.return_value = mock_response

        with self.assertRaises(KeyError):
            get_access_token("test-project", "test-api-key")


if __name__ == "__main__":
    unittest.main()
