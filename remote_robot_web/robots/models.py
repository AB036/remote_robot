from django.db import models


class Robot(models.Model):
    id = models.PositiveIntegerField(
        primary_key=True,
        help_text="The robots's ID, given automatically by the server.",
    )
    name = models.CharField(
        max_length=100, null=True,
        help_text="The robots's name given by the user.",
    )
    ip_address = models.CharField(
        max_length=100, null=True,
        help_text="The robots's ip address to communicate with it and given by the user.",
    )
