from django.urls import path
from django.conf.urls import url

from . import views  # import views so we can use them in urls.

urlpatterns = [
    path('', views.ControlBoardView.as_view(), name='index'),
    url(r'^ajax/move/$', views.move, name="ajax_moveUp")
]
