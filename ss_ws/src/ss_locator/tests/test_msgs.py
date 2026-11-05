from ss_msgs.action import LocatePanels
from ss_msgs.msg import CleanupDone, LocatorStatus, PanelTop


def test_panel_top_fields():
    assert PanelTop.get_fields_and_field_types() == {
        "x_m": "double",
        "y_m": "double",
        "top_height_m": "float",
        "height_measured": "boolean",
        "yaw_rad": "double",
        "width_m": "float",
        "depth_m": "float",
        "pos_std_m": "float",
        "yaw_std_deg": "float",
        "lugs": "int32",
        "front_known": "boolean",
        "front_yaw_rad": "double",
    }


def test_cleanup_done_fields():
    assert CleanupDone.get_fields_and_field_types() == {
        "header": "std_msgs/Header",
        "goal_id": "unique_identifier_msgs/UUID",
        "reason": "string",
        "lift_restored": "boolean",
    }


def test_locator_status_fields():
    assert LocatorStatus.get_fields_and_field_types() == {
        "header": "std_msgs/Header",
        "interface_version": "string",
        "phase": "string",
        "goal_id": "unique_identifier_msgs/UUID",
        "last_reject_reason": "string",
    }


def test_locate_panels_fields():
    assert LocatePanels.Goal.get_fields_and_field_types() == {
        "azimuth_from_rad": "float",
        "azimuth_span_rad": "float",
        "max_distance_m": "float",
        "time_limit_s": "float",
    }
    assert LocatePanels.Result.get_fields_and_field_types() == {
        "header": "std_msgs/Header",
        "reason": "string",
        "panels": "sequence<ss_msgs/PanelTop>",
        "seen_from_rad": "sequence<float>",
        "seen_span_rad": "sequence<float>",
    }
    assert LocatePanels.Feedback.get_fields_and_field_types() == {"phase": "string"}
