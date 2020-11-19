<template>
  <div class="machine-interface">
    <!-- Machine Image -->
    <div class="machine-container">
      <div v-if="displayOption == 'machine'">
        <MachineDisplay 
          :machineOn="machineOn"
          :temperature="response.temperature"
          :temperature_setpoint="response.temperature_setpoint"
          :pressure="response.pressure"
          :mass="response.mass"
          :brew_time="t_brew"
        />
      </div>
      <div v-if="displayOption == 'graph'">
        <div v-if="loggedData.current.length == 0" style="display: flex; justify-content: center; align-items: center; min-width: 350px">
          <!-- <single-response-chart :data="watchedData"></single-response-chart> -->
          <v-btn color="secondary" @click="viewLastSession">Last Session</v-btn>
        </div>
        <div v-else>
          <single-response-chart :data="loggedData"></single-response-chart>
        </div>
      </div>
    </div>
    <br>
    <!-- Controls -->
    <v-row align="center">
      <v-col cols="auto">
        <v-switch color="secondary" value :input-value="machineOn" @change="toggleOnOff" :label="`${machineOn ? 'On' : 'Off'}`"></v-switch>
      </v-col>
      <v-spacer></v-spacer>
      <v-col cols="auto" class="px-1" v-if="machineOn">
        <div v-if="machineBrewing">
          <v-btn color="error" fab small elevation="1" @click="toggleBrew"><v-icon>mdi-coffee</v-icon></v-btn>
        </div>
        <div v-else>
          <v-btn color="info" fab small elevation="1" @click="toggleBrew"><v-icon>mdi-coffee</v-icon></v-btn>
        </div>
      </v-col>
      <v-col cols="auto" class="px-1" v-if="machineOn">
        <div v-if="machineMode==3">
          <v-btn color="error" fab small elevation="1" @click="toggleClean"><v-icon>mdi-dishwasher-off</v-icon></v-btn>
        </div>
        <div v-else>
          <v-btn color="secondary" fab small elevation="1" @click="toggleClean"><v-icon>mdi-dishwasher</v-icon></v-btn>
        </div>
      </v-col>
      <v-col cols="auto" class="px-1">
        <v-btn v-if="machineOn" fab small outlined :color="waterLevelColor">
            <v-icon>mdi-water</v-icon>
        </v-btn>
      </v-col>
      <v-col cols="auto" class="px-1">
        <v-btn color="secondary" @click="changeDisplay" fab small elevation="1" outlined>
          <div v-if="displayOption == 'machine'">
            <v-icon>mdi-chart-line</v-icon>
          </div>
          <div v-else>
            <v-icon>mdi-file-presentation-box</v-icon>
          </div>
        </v-btn>
      </v-col>
      <v-col cols="auto" class="px-1">
         <v-btn color="secondary" @click="toggleOverride" fab small elevation="1" outlined>
          <v-icon>mdi-wrench</v-icon>
        </v-btn>
      </v-col>
    </v-row>

    <div v-if="machineBrewing">
      <v-row align="center">
        <v-progress-linear :value="brewProgress" color="blue-grey" height="25" rounded>
          <div v-if="response.mass == null">
            No scale detected
          </div>
          <div v-else>
            {{ response.mass | temperatureDisplayFilter }}g / {{ response.mass_setpoint }}g
          </div>
        </v-progress-linear>
      </v-row>
    </div>

    <div v-if="machineMode == 2  && !machineBrewing">
      <v-row align="center">
        <v-col cols="auto" class="px-1" align-self="baseline">
          <v-text-field v-model="heaterDuty" type="number" suffix="%" style="max-width: 70px" label="Heater duty"></v-text-field>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-btn color="secondary" @click="sendHeaterDuty">
            Heater
          </v-btn>
        </v-col>
        <v-spacer />
        <v-col cols="auto" class="px-1" align-self="baseline">
          <v-text-field v-model="pumpDuty" type="number" suffix="%" style="max-width: 70px" label="Pump duty"></v-text-field>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-btn color="secondary" @click="sendPumpDuty">
            Pump
          </v-btn>
        </v-col>
        <!-- <v-col cols="auto" class="px-1">
          <v-chip color="info">
            <v-avatar left color="info darken-1">Kp</v-avatar>{{ heater.kp }}
          </v-chip>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-chip color="info">
            <v-avatar left color="info darken-1">Ki</v-avatar>{{ heater.ki }}
          </v-chip>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-chip color="info">
            <v-avatar left color="info darken-1">Kd</v-avatar>{{ heater.kd }}
          </v-chip>
        </v-col> -->
      </v-row>
    </div>

  </div>
</template>

<script>
import MachineDisplay from '@/components/MachineDisplay.vue'
import SingleResponseChart from '@/components/SingleResponseChart.vue'
import apiMixin from '@/mixins/apiMixin'
import { eventBus } from '@/main'
import { ros } from '@/ros'
import axios from 'axios'
import ROSLIB from 'roslib'

export default {
  name: 'MachineInterface',
  mixins: [apiMixin],
  components: {
    MachineDisplay,
    SingleResponseChart
  },
  data: function () {
    return {
      response: {
        temperature: 0,
        temperature_setpoint: 0,
        heater_duty: 0,
        pressure: 0,
        pressure_setpoint: 0,
        pump_duty: 0,
        mass: 0,
        mass_setpoint: 0,
        low_water: false
      },
      heaterDuty: 100,
      pumpDuty: 100,
      displayOption: 'machine',
      n_datapoints: 500,
      loggedData: {'current': []},
      logIntervalReference: null,
      t_brew: 0
    }
  },
  props: {
    machineOn: Boolean,
    machineBrewing: Boolean,
    machineMode: Number
  },
  computed: {
    brewProgress: function () {
      return 100 * this.response.mass / this.response.mass_setpoint
    },
    waterLevelColor: function () {
      return this.response.low_water ? 'error' : 'success'
    }
  },
  methods: {
    changeDisplay: function () {
      if (this.displayOption === 'machine') {
        this.displayOption = 'graph'
      } else {
        this.displayOption = 'machine'
      }
    },
    toggleOnOff () {
      const mode = (this.machineMode === 0) ? 1 : 0
      eventBus.$emit('changeMode', mode)
    },
    toggleBrew () {
      eventBus.$emit('toggleBrew')
    },
    toggleOverride () {
      const mode = (this.machineMode != 2) ? 2 : 1
      eventBus.$emit('changeMode', mode)
    },
    toggleClean () {
      const mode = (this.machineMode != 3) ? 3 : 1
      eventBus.$emit('changeMode', mode)
    },
    sendHeaterDuty () {
      const getParams = {
        params: {
          duty: this.heaterDuty
        }
      }
      axios.get('/api/v1/heaterduty/', getParams)
        .then(response => {})
        .catch(error => console.log(error))
    },
    sendPumpDuty () {
      const getParams = {
        params: {
          duty: this.pumpDuty
        }
      }
      axios.get('/api/v1/pumpduty/', getParams)
        .then(response => {})
        .catch(error => console.log(error))
    },
    viewLastSession () {
      axios.get('/api/v1/session/')
        .then(response => {
          const lastSession = response.data[response.data.length - 1]
          this.$router.push({ name: 'Session', params: { sessionIds: lastSession.id.toString() } })
        })
        .catch(error => console.log(error))
    },
    logAtInterval () {
      const dataset = Object.assign({}, this.response)
      dataset.t = new Date()
      this.loggedData.current.push(dataset)
      while (this.loggedData.length > this.n_datapoints) {
        this.loggedData.pop(0)
      }
    },
  },
  created () {
    // Fire event to check on/off status
    eventBus.$emit('updateStatus')
    // Log data at interval
    this.logIntervalReference = setInterval(() => { this.logAtInterval() }, 500)
    // Subscribers
    var heater_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/heater_controller',
      messageType : 'django_interface/SilviaController'
    });
    heater_listener.subscribe((message) => {
      // console.log('Received message on ' + heater_listener.name + ': T=' + message.input);
      this.response.temperature = Number(message.input)
      this.response.temperature_setpoint = Number(message.setpoint)
      this.response.heater_duty = Number(message.output)
    });
    var pump_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/pump_controller',
      messageType : 'django_interface/SilviaController'
    });
    pump_listener.subscribe((message) => {
      // console.log('Received message on ' + pump_listener.name + ': P=' + message.input);
      this.pressure = Number(message.input)
      this.pressure_setpoint = Number(message.setpoint)
      this.pump_duty = Number(message.output)
    });
    var scale_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/scale',
      messageType : 'django_interface/SilviaMass'
    });
    scale_listener.subscribe((message) => {
      // console.log('Received message on ' + scale_listener.name + ': T=' + message.input);
      this.response.mass = Number(message.input)
      this.response.mass_setpoint = Number(message.setpoint)
    });
    var water_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/low_water',
      messageType : 'django_interface/SilviaWater'
    });
    water_listener.subscribe((message) => {
      // console.log('Received message on ' + scale_listener.name + ': T=' + message.input);
      this.response.low_water = Boolean(message.low_water)
    });
    var timer_listener = new ROSLIB.Topic({
      ros : ros,
      name : '/brew_duration',
      messageType : 'django_interface/SilviaBrewTimer'
    });
    timer_listener.subscribe((message) => {
      this.t_brew = Number(message.duration.secs)
    });
  },
  destroyed () {
    // console.log('Cancel temperature update')
    // clearInterval(this.intervalReference)
  }
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.machine-interface {
  margin: auto;
}

.machine-container {
  position: relative;
}
</style>
