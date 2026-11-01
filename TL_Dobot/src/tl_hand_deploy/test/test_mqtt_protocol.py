import json

import pytest

from tl_hand_deploy.mqtt_protocol import (
    RET_FAILED,
    RET_OCCUPIED,
    RET_SUCCESS,
    encode_feedback,
    parse_command,
    parse_target_map,
)


def test_parse_commands_and_target_ids() -> None:
    assert parse_command('{"cmd":"seed","targetId":"A1-01"}').target_id == "A1-01"
    assert parse_command('{"cmd":"trim_pick","targetId":1}').target_id == "1"


def test_parse_command_rejects_missing_or_unknown_fields() -> None:
    with pytest.raises(ValueError):
        parse_command('{"cmd":"unknown","targetId":1}')
    with pytest.raises(ValueError):
        parse_command('{"cmd":"seed"}')
    with pytest.raises(ValueError):
        parse_command(b"\xff")


def test_feedback_matches_robot_protocol() -> None:
    assert json.loads(encode_feedback("seed", RET_SUCCESS)) == {
        "cmd": "seed",
        "retCode": 0,
    }
    assert json.loads(encode_feedback("trim_place", RET_FAILED))["retCode"] == 1
    assert json.loads(encode_feedback("trim_pick", RET_OCCUPIED))["retCode"] == 2


def test_parse_target_map() -> None:
    assert parse_target_map('{"A1-01": 1, "A1-02": 2}') == {
        "A1-01": 1,
        "A1-02": 2,
    }
    with pytest.raises(ValueError):
        parse_target_map('{"A1-01": 7}')
    with pytest.raises(ValueError):
        parse_target_map('{"A1-01": "basket-one"}')
