<template>
  <v-app>
    <AppNaviagtion :machineOn="machineOn" />

    <v-content>
      <v-container fluid class="mt-5">
        <v-row align="center" justify="center">
          <router-view :machineOn="machineOn" :machineBrewing="machineBrewing" :machineMode="machineMode"></router-view>
        </v-row>
      </v-container>
    </v-content>

    <v-footer app>
      <!-- -->
    </v-footer>
  </v-app>
</template>

<script>
import axios from 'axios'
// import { axiosApi } from '@/api'
import { eventBus } from '@/main'
import { ros } from '@/ros'
import AppNaviagtion from './components/AppNavigation'
import ROSLIB from 'roslib'

// Subscribers
var status_listener = new ROSLIB.Topic({
  ros : ros,
  name : '/status',
  messageType : 'django_interface/SilviaStatus'
});

axios.defaults.xsrfCookieName = 'csrftoken'
axios.defaults.xsrfHeaderName = 'X-CSRFToken'
axios.defaults.headers['Content-Type'] = 'application/json'
axios.defaults.withCredentials = true
axios.defaults.trailingSlash = true
axios.interceptors.request.use((config) => {
  if (config.addTrailingSlash && config.url[config.url.length - 1] !== '/') {
    config.url += '/'
  }
  return config
})

export default {
  name: 'App',

  components: {
    AppNaviagtion
  },

  data: function () {
    return {
      machineOn: false,
      machineBrewing: false,
      machineMode: 0,
      // Modes
      MODE_IGNORE: -1,
      MODE_OFF: 0,
      MODE_PID: 1,
      MODE_MANUAL: 2,
      MODE_CLEAN: 3
    }
  },

  watch: {
    // Change tab name in browser
    '$route' (to, from) {
      document.title = to.meta.title || 'Silvia'
    }
  },

  created () {
    status_listener.subscribe((message) => {
      console.log('Received message on ' + status_listener.name + ': ' + message.mode);
      this.machineBrewing = Boolean(message.brew)
      this.machineMode = Number(message.mode)
      this.machineOn = (message.mode == this.MODE_OFF) ? false : true 
    });

    // Handle machine brew on/off globally
    eventBus.$on('toggleBrew', () => {
      const axiosData = {
        id: 1,
        mode: this.MODE_IGNORE,
        brew: !this.machineBrewing
      }

      axios.put('/api/v1/status/1/', axiosData)
        .then(response => {
          console.log(response)
          // setTimeout(() => { eventBus.$emit('updateStatus') }, 1000)
          // eventBus.$emit('updateStatus')
        })
        .catch(error => console.log(error))
    })

    // Handle mode change globally
    eventBus.$on('changeMode', (mode) => {
      const axiosData = {
        id: 1,
        brew: this.machineBrewing,
        mode: mode
      }

      axios.put('/api/v1/status/1/', axiosData)
        .then(response => {
          console.log(response)
          // setTimeout(() => { eventBus.$emit('updateStatus') }, 1000)
          // eventBus.$emit('updateStatus')
        })
        .catch(error => console.log(error))
    })

    eventBus.$on('updateStatus', () => {
      axios.get('/api/v1/status/1/')
        .then(response => {
          console.log('Updating status')
          this.machineBrewing = Boolean(response.data.brew)
          this.machineMode = Number(response.data.mode)
          this.machineOn = (response.data.mode == this.MODE_OFF) ? false : true 
        })
        .catch(error => console.log(error))
    })
  }
}
</script>
