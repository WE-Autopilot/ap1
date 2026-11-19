"""Runs the tests in a ROS domain that no other test run on this machine uses.

The YOLO node the tests launch must not talk to nodes that are already running,
such as a deploy container or a teammate's nodes. domain_coordinator reserves
an ID from 1 to 100 (ROS_DOMAIN_ID defaults to 0) for the whole pytest run.
"""
import contextlib
import os

import domain_coordinator


def pytest_configure(config):
    # A hook rather than a fixture: launch_testing tests don't request fixtures,
    # and launched nodes inherit ROS_DOMAIN_ID from this process's environment.
    stack = contextlib.ExitStack()
    os.environ['ROS_DOMAIN_ID'] = str(stack.enter_context(domain_coordinator.domain_id()))
    config.add_cleanup(stack.close)
