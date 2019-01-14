from django.test import TestCase, Client
from django.urls import reverse


# Create your tests here.

class ControlBoardViewTests(TestCase):
    def test_wrong_command_does_nothing(self):
        """
        If a wrong command (from keyboard for instance) is used, it doesn't do anything.
        """
        client = Client()
        params = {'direction': 'something'}

        response = client.get(reverse('ajax_move'), params)
        self.assertEqual(response.status_code, 404)

    def test_command_is_correctly_received(self):
        """
        When a command is used (either from keyboard or mouse click), it is correctly received.
        """
        client = Client()
        params = {'direction': 'up'}

        response = client.get(reverse('ajax_move'), params)
        self.assertEqual(response.status_code, 200)
        self.assertJSONEqual(str(response.content, encoding='utf8'), params)

    def test_socket_has_proper_format(self):
        """
        The socket with the order is readable and has the proper format.
        """
        return NotImplementedError
