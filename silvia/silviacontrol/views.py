from django.shortcuts import render
from django.http import HttpResponse
from django.utils import timezone
from django.conf import settings
from rest_framework import viewsets, generics, views
from rest_framework.response import Response
from rest_framework.decorators import action
from .models import (SettingsModel, SessionModel,
                        ResponseModel, ScheduleModel)
from .serializers import (SettingsSerializer, StatusSerializer, SessionSerializer,
                            ResponseSerializer, ScheduleSerializer)
from .utils import debug_log
import json
from .tasks import async_ros_set_heater, sync_ros_get_status, async_ros_set_status

# Html Views -----------
def index(request):
    return HttpResponse("You're at Silvia Mission Control")

# API Views -----------
class SettingsViewSet(viewsets.ModelViewSet):
    """
    API endpoint that allows settings to be viewed or edited
    """
    queryset = SettingsModel.objects.all()
    serializer_class = SettingsSerializer

    def get_object(self):
          if self.request.method == 'PUT':
              obj, created = SettingsModel.objects.get_or_create(pk=1)
              return obj
          else:
              return super(SettingsViewSet, self).get_object()


class ResponseViewSet(viewsets.ModelViewSet):
    """
    API endpoint for machine responses to be viewed or edited
    """
    serializer_class = ResponseSerializer
    queryset = ResponseModel.objects.all()

    def get_object(self):
        # Override get_object to see if request is for latest object
        if self.kwargs['pk'] == 'latest':
            try:
                response = ResponseModel.objects.order_by('-t')[0]
                # if (timezone.now() - response.t).total_seconds() > 10:
                #     response = None
                return response
            except IndexError as e:
                return None
        else:
            return super(ResponseViewSet, self).get_object()

    def get_queryset(self):
        queryset = ResponseModel.objects.all()
        session_id = self.request.query_params.get('session', None)
        
        if session_id is not None:
            session = SessionModel.objects.get(id=session_id)
            queryset = ResponseModel.objects.filter(t__range=(session.t_start, session.t_end))

        return queryset

    @action(detail=False, methods=['get'])
    def sessions(self, request):
        """
        Query responses for multiple sessions
        """
        session_ids_string = request.query_params.get('session', None)
        
        if session_ids_string == "active":
            try:
                session = SessionModel.objects.filter(active=True).order_by('-t_start')[0]
                if session.t_end is None:
                    queryset = ResponseModel.objects.filter(t__range=(session.t_start, timezone.now()))
                else:
                    queryset = ResponseModel.objects.filter(t__range=(session.t_start, session.t_end))
                # Take last 50 points
                queryset_dict = {session.id: self.serializer_class(queryset, many=True).data[-50:]}
                return Response(queryset_dict)
            except IndexError as e:
                return Response(data=None)

        if session_ids_string is not None:
            session_ids = [int(x) for x in session_ids_string.split(',')]
            queryset_dict = {}
            for session_id in session_ids:
                session = SessionModel.objects.get(id=session_id)
                if session.t_end is None:
                    queryset = ResponseModel.objects.filter(t__range=(session.t_start, timezone.now()))
                else:
                    queryset = ResponseModel.objects.filter(t__range=(session.t_start, session.t_end))
                queryset_dict[session_id] = self.serializer_class(queryset, many=True).data
            return Response(queryset_dict)
        return ResponseModel.objects.all()


class SessionViewSet(viewsets.ModelViewSet):
    """
    API endpoint for on/off sessions to be viewed or edited
    """
    queryset = SessionModel.objects.all()
    serializer_class = SessionSerializer       


class ScheduleViewSet(viewsets.ModelViewSet):
    """
    API endpoint for machine schedules to be viewed or edited
    """
    queryset = ScheduleModel.objects.all()
    serializer_class = ScheduleSerializer

class ManualControlView(views.APIView):
    """
    Non-model based view for turning the heater on and off manually
    """
    def get(self, request, format=None):
        """
        Turn heater on/off
        """
        debug_log("Manual control GET request")
        duty = float(self.request.query_params.get('duty', 0))
        async_ros_set_heater.delay(duty=duty)
        return Response({"duty": duty})

def string2bool(input_string):
    """
    Converts string to boolena by checking if texts meaning is True, otherwise returns False.
    """
    return input_string in ['true', 'True', 'Yes', '1']

class StatusView(views.APIView):
    """
    Non-model based view for returning machine status via ROS
    """
    def get(self, request, format=None):
        """
        Get machine status
        """
        debug_log("Machine status GET request")
        status_response = sync_ros_get_status()
        return Response(status_response)

    def put(self, request, format=None):
        """
        Set machine status
        """
        debug_log("Machine status PUT request")
        async_ros_set_status.delay()
        return Response()
