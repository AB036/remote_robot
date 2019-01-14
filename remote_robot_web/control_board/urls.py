from django.urls import path
from django.conf.urls import url

from . import views

urlpatterns = [
    path('', views.ControlBoardView.as_view(), name='index'),
    url(r'^ajax/move/$', views.move, name="ajax_move")
]
