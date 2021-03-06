from django.core.management.base import BaseCommand, CommandError
from silviacontrol.models import ResponseModel, SessionModel
from django.conf import settings as django_settings
from silviacontrol.utils import debug_log

class Command(BaseCommand):
    help = 'Cleans erraneous database entries'

    def handle(self, *args, **options):
        # Responses
        n_response_errors = 0
        responses = ResponseModel.objects.all()
        for response in responses:
            if response.temperature is None or response.pressure is None or response.t is None:
                n_response_errors += 1
                response.delete()
        print("{} responses removed due to errors".format(n_response_errors))

        # Sessions
        n_session_errors = 0
        sessions = SessionModel.objects.all()
        for session in sessions:
            if (session.t_start is None) or (session.t_end is None) or (session.t_end < session.t_start):
                n_session_errors += 1
                session.delete()
        print("{} sessions removed due to errors".format(n_session_errors))

