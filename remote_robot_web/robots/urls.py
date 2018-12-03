from django.urls import path

from . import views

urlpatterns = [
    path('robots/<str:id>', views.StreamingVideoView.as_view(), name='streaming'),
]
