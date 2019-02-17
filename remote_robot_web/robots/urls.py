from django.urls import path

from . import views

urlpatterns = [
    path('webcam', views.StreamingVideoView.as_view(), name='streaming_webcam'),
]
