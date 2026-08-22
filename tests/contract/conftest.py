"""Shared pytest fixtures for tests/contract/*.py — auto-discovered by
pytest for every test module in this directory, so individual test files
no longer need to redefine ros_context themselves.
"""
import pytest
import rclpy


@pytest.fixture(scope='module', autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()
