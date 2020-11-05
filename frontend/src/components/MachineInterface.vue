<template>
  <div class="machine-interface">
    <!-- Machine Image -->
    <div class="machine-container">
      <div v-if="displayOption == 'machine'">
        <MachineDisplay :machineOn="machineOn" :temperature="temperature" />
      </div>
      <div v-if="displayOption == 'graph'">
        <div v-if="sessionData == null" style="display: flex; justify-content: center; align-items: center; min-width: 350px">
          <!-- <single-response-chart :data="watchedData"></single-response-chart> -->
          <v-btn color="secondary" @click="viewLastSession">Last Session</v-btn>
        </div>
        <div v-else>
          <single-response-chart :data="sessionData"></single-response-chart>
        </div>
      </div>
      <v-btn v-if="displayOption == 'machine'" id="temp-btn" outlined :color="tempBtnColor" @click="changeDisplay">
        <div v-if="temperature == null">-</div>
        <div v-else>{{ temperature | temperatureDisplayFilter }}&#8451;</div>
      </v-btn>
      <!-- <v-btn v-if="machineOn" id="water-btn" fab small outlined :color="waterLevelColor">
          <v-icon>mdi-water</v-icon>
      </v-btn> -->
      <v-btn v-if="machineBrewing" id="brew-btn" class="" outlined text color="secondary">
        <v-col>
          <v-row class="pb-1" justify="center">{{ mass | temperatureDisplayFilter }}g</v-row>
          <v-row class="" justify="center">{{ t_elapsed }}s</v-row>
        </v-col>
      </v-btn>
    </div>
    <br>
    <!-- Controls -->
    <v-row align="center">
      <!-- <v-btn color="secondary" @click="toggleOnOff">{{ machineOn ? "On" : "Off" }}</v-btn> -->
      <v-col cols="auto">
        <v-switch color="secondary" value :input-value="machineOn" @change="toggleOnOff" :label="`${machineOn ? 'On' : 'Off'}`"></v-switch>
      </v-col>
      <v-spacer></v-spacer>
      <v-col cols="auto" class="px-1" v-if="machineOn">
        <div v-if="machineBrewing">
          <v-btn color="error" @click="toggleBrew">Cancel</v-btn>
        </div>
        <div v-else>
          <v-btn color="accent lighten-1" @click="toggleBrew">Brew</v-btn>
        </div>
      </v-col>
      <v-col cols="auto" class="px-1">
        <v-btn v-if="machineOn" fab small outlined :color="waterLevelColor">
            <v-icon>mdi-water</v-icon>
        </v-btn>
      </v-col>
      <v-col cols="auto" class="px-1">
        <v-btn :color="tempBtnColor" @click="changeDisplay" fab small elevation="1" outlined>
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

    <div v-if="machineOn">
      <v-row align="center">
        <v-progress-linear :value="brewProgress" color="blue-grey" height="25" rounded>
          <div v-if="mass == null">
            No scale detected
          </div>
          <div v-else>
            {{ mass | temperatureDisplayFilter }}g / {{ mass_setpoint }}g
          </div>
        </v-progress-linear>
      </v-row>
    </div>

    <div v-if="machineMode != 0">
      <v-row align="center">
        <v-col cols="auto" class="px-1" align-self="baseline">
          <v-text-field v-model="dutyOverride" type="number" suffix="%" style="max-width: 70px"></v-text-field>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-btn color="secondary" @click="toggleHeaterOn">
            Heat On
          </v-btn>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-btn color="secondary" @click="toggleHeaterOff">
            Off
          </v-btn>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-btn color="accent lighten-1" @click="toggleClean">
            <div v-if="machineMode == 3">
              Cancel Clean
            </div>
            <div v-else>
              Run Clean
            </div>
          </v-btn>
        </v-col>
        <!-- <v-col cols="auto" class="px-1">
          <v-btn color="accent lighten-1" @click="toggleAutoTune" disabled>
            <div v-if="machineMode == 2">
              Cancel Tuning
            </div>
            <div v-else>
              Auto Tune
            </div>
          </v-btn>
        </v-col> -->
        <v-col cols="auto" class="px-1">
          <v-chip color="info">
            <v-avatar left color="info darken-1">Kp</v-avatar>{{ heater_kp }}
          </v-chip>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-chip color="info">
            <v-avatar left color="info darken-1">Ki</v-avatar>{{ heater_ki }}
          </v-chip>
        </v-col>
        <v-col cols="auto" class="px-1">
          <v-chip color="info">
            <v-avatar left color="info darken-1">Kd</v-avatar>{{ heater_kd }}
          </v-chip>
        </v-col>
      </v-row>
    </div>

  </div>
</template>

<script>
import MachineDisplay from '@/components/MachineDisplay.vue'
import SingleResponseChart from '@/components/SingleResponseChart.vue'
import apiMixin from '@/mixins/apiMixin'
import { eventBus } from '@/main'
import axios from 'axios'

export default {
  name: 'MachineInterface',
  mixins: [apiMixin],
  components: {
    MachineDisplay,
    SingleResponseChart
  },
  data: function () {
    return {
      temperature: 0,
      temperature_setpoint: 60,
      mass: null, // Brewed coffee mass (g)
      mass_setpoint: 20,
      dutyOverride: 100,
      heater_kp: 0,
      heater_ki: 0,
      heater_kd: 0,
      low_water: false,
      t_update: 10,
      displayOption: 'machine',
      intervalReference: null, // Varibale to hold setInterval for getting temperature,
      n_datapoints: 10,
      sessionData: null,
    }
  },
  props: {
    machineOn: Boolean,
    machineBrewing: Boolean,
    machineMode: Number
  },
  computed: {
    tempBtnColor: function () {
      if (Math.abs(this.temperature_setpoint - this.temperature) < 2) {
        return 'success'
      }
      return 'secondary'
    },
    brewProgress: function () {
      return 100 * this.mass / this.mass_setpoint
    },
    waterLevelColor: function () {
      return this.low_water ? 'error' : 'success'
    },
    t_elapsed: function () {
      return 0
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
      eventBus.$emit('toggleOnOff', mode)
    },
    toggleBrew () {
      eventBus.$emit('toggleBrew')
    },
    toggleOverride () {
      const mode = (this.machineMode === 0) ? 2 : 0
      eventBus.$emit('changeMode', mode)
    },
    toggleHeaterOn () {
      const getParams = {
        params: {
          duty: this.dutyOverride
        }
      }
      axios.get('/api/v1/override/', getParams)
        .then(response => {})
        .catch(error => console.log(error))
    },
    toggleHeaterOff () {
      const getParams = {
        params: {
          duty: 0
        }
      }
      axios.get('/api/v1/override/', getParams)
        .then(response => {})
        .catch(error => console.log(error))
    },
    toggleClean () {
      if (this.machineMode === 3) {
        eventBus.$emit('changeMode', 2) // Manual
      } else {
        eventBus.$emit('changeMode', 3) // Clean
      }
    },
    updateResponse () {
      eventBus.$emit('updateStatus')
      if (this.machineOn) {
        // Get all responses from current session
        const getParams = { params: { session: 'active' } }

        axios.get('/api/v1/response/sessions/', getParams)
          .then(response => {
            // console.log(response.data)
            // this.sessionData = response.data
            if (response.data === '') {
              // console.log('No response data returned')
              return false
            }
            this.sessionData = Object.assign({}, this.sessionData, response.data)
            const latestSession = response.data[Object.keys(response.data)[0]]
            if (latestSession === undefined || latestSession.length === 0) {
              // console.log('No logged responses yet')
              return false
            }
            const lastResponse = latestSession[latestSession.length - 1]
            this.temperature = lastResponse.temperature
            this.mass = lastResponse.mass
            this.low_water = lastResponse.low_water
          })
          .catch(error => console.log(error))
      } else { // If machine off just get temperature
        // Get latest response only
        axios.get('/api/v1/response/latest/')
          .then(response => {
            // console.log(response.data)
            // Check if temperature is old
            const deltaTime = (new Date() - new Date(response.data.t)) / 1000
            // console.log(deltaTime)
            if (deltaTime > 15) {
              this.temperature = null
            } else {
              this.temperature = response.data.temperature
            }
            this.mass = response.data.mass
            this.low_water = response.data.low_water
          })
          .catch(error => console.log(error))
        this.sessionData = null
      }
    },
    updateInterval () {
      axios.get('/api/v1/settings/1/')
        .then(response => {
          this.mass_setpoint = response.data.mass
          this.temperature_setpoint = response.data.temperature_setpoint
          this.heater_kp = response.data.heater_kp
          this.heater_ki = response.data.heater_ki
          this.heater_kd = response.data.heater_kd
          this.intervalReference = setInterval(() => {
            this.updateResponse()
          }, 1000 * this.t_update)
        })
        .catch(error => console.log(error))
    },
    viewLastSession () {
      axios.get('/api/v1/session/')
        .then(response => {
          const lastSession = response.data[response.data.length - 1]
          this.$router.push({ name: 'Session', params: { sessionIds: lastSession.id.toString() } })
        })
        .catch(error => console.log(error))
    }
  },
  created () {
    this.updateInterval()
    this.updateResponse()
    // Fire event to check on/off status
    eventBus.$emit('updateStatus')
  },
  destroyed () {
    // console.log('Cancel temperature update')
    clearInterval(this.intervalReference)
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

#temp-btn {
  position: absolute;
  top: 29%;
  left: 50%;
  transform: translate(-50%, -50%);
  -ms-transform: translate(-50%, -50%);
  background-color: rgb(236, 236, 236);
}

#water-btn {
  position: absolute;
  /* top: 10%;
  left: 83%; */
  /* top: 92.1%; */
  top: 20%;
  left: 50%;
  transform: translate(-50%, -50%);
  -ms-transform: translate(-50%, -50%);
  background-color: rgb(236, 236, 236);
}

#brew-btn {
  position: absolute;
  /* top: 10%;
  left: 83%; */
  top: 75%;
  left: 50%;
  transform: translate(-50%, -50%);
  -ms-transform: translate(-50%, -50%);
  background-color: rgb(236, 236, 236);
  height: 50px;
  text-transform: none !important;
}
</style>
