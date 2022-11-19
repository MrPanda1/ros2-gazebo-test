#!/usr/bin/env python3

import unittest
import os

import launch
import launch.actions
import launch_testing.actions
import launch_testing.markers
import pytest


# This function specifies the processes to be run for our test
@pytest.mark.launch_test
def generate_test_description():
    """Launch a simple process to see if the bot is moving."""
    # Get current directory
    dir_path = os.path.dirname(os.path.realpath(__file__))
    
    # This is necessary to get unbuffered output from the process under test
    proc_env = os.environ.copy()
    proc_env['PYTHONUNBUFFERED'] = '1'
    
    test_does_bot_move = launch.actions.ExecuteProcess(
        cmd=[f'{dir_path}/test_bot_moves.py'],
        shell=True,
        env=proc_env,
        output='screen'
    )
    
    return launch.LaunchDescription([
        test_does_bot_move,
        # Tell launch to start the test
        launch_testing.actions.ReadyToTest()
    ]), {'test_does_bot_move': test_does_bot_move}

# These tests will run concurrently with the test_does_bot_move process.
# After all these tests are done, the launch system will shut down the processes that it started up
class TestOutput(unittest.TestCase):
    def test_true(self, proc_output):
        proc_output.assertWaitFor('True', timeout=5, stream='stdout')

