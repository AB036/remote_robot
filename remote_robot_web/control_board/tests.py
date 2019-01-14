from django.test import TestCase, Client
from django.test.client import RequestFactory

# Create your tests here.

class ControlBoardViewTests(TestCase):
    def setUp(self):
        # Every test needs access to the request factory.
        self.factory = RequestFactory()

    def test_wrong_command_does_nothing(self):
        """
        If a wrong command (from keyboard for instance) is used, it doesn't do anything.
        """
        client = Client()
        action_url = 'control_board/ajax/move/'
        order = 'something'

        response = client.post(action_url, {'direction': order})
        print(response.status_code)

    def test_command_is_correctly_received(self):
        """
        When a command is used (either from keyboard or mouse click), it is correctly received.
        """
        return NotImplementedError

    def test_socket_has_proper_format(self):
        """
        The socket with the order is readable and has the proper format.
        """
        return NotImplementedError