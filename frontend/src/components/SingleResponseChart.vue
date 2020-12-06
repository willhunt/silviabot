<template>
  <div>
    <v-card class="mb-4 mx-2" :width="chartWidth" min-width="350" max-width="1000" style="position: relative;">
      <response-chart class="px-6 pt-6 pb-1" :chartData="graphData" :chartOptions="graphOptions"></response-chart>
      <v-card-actions>
        <v-spacer></v-spacer>
        <v-btn color="secondary" @click="graphBrewOnly = !graphBrewOnly">isolate brew</v-btn>
        <v-btn color="info" @click="dataHidden=!dataHidden">data</v-btn>
      </v-card-actions>
    </v-card>

    <v-card v-if="!dataHidden" class="mx-auto mb-4" min-width=750 max-width=1600>
      <v-data-table class="mx-4"
        :headers="headers"
        :items="tableData"
        dense>
      </v-data-table>
      <v-card-actions>
        <v-spacer></v-spacer>
        <v-btn color="secondary" @click="exportData">export</v-btn>
      </v-card-actions>
    </v-card>
  </div>
</template>

<script>
import ResponseChart from '@/components/ResponseChart.vue'

export default {
  name: 'SingleResponseChart',
  components: {
    ResponseChart
  },
  data: function () {
    return {
      graphBrewOnly: false,
      chartWidth: 0.8 * window.innerWidth,
      graphOptions: {
        scales: {
          xAxes: [{
            id: 'time-axis',
            position: 'bottom',
            display: true,
            scaleLabel: {
              display: true,
              labelString: 'Time [s]'
            },
            ticks: {
              beginAtZero: true
            }
          }],
          yAxes: [{
            id: 'temperature-axis',
            position: 'left',
            display: true,
            scaleLabel: {
              display: true,
              labelString: ['Temperature [C]', 'Duty [%]']
            },
            ticks: {
              beginAtZero: true,
              suggestedMin: 0,
              suggestedMax: 110,
              fontColor: '#ff5a5f'
            }
          },
          {
            id: 'mass-axis',
            position: 'right',
            display: true,
            scaleLabel: {
              display: true,
              labelString: 'Mass (g)'
            },
            ticks: {
              suggestedMin: 0,
              suggestedMax: 30,
              fontColor: '#7fd1b9'
            }
          },
          {
            id: 'pressure-axis',
            position: 'right',
            display: true,
            scaleLabel: {
              display: true,
              labelString: 'Pressure (bar)'
            },
            ticks: {
              suggestedMin: 0,
              suggestedMax: 10,
              fontColor: '#769fb6'
            }
          }]
        },
        showLines: true,
        maintainAspectRatio: false,
        responsive: true
      },
      dataHidden: true,
      headers: [
        { text: 'Time Stamp', value: 'ts' },
        { text: 'Time', value: 't' },
        { text: 'Duty', value: 'heater_duty' },
        { text: 'Temperature', value: 'temperature' }
      ]
    }
  },
  props: {
    data: {},
    brewing: Boolean
  },
  computed: {
    graphData: function () {
      const graphData = {}
      if (this.data != null) {
        graphData.datasets = []
        const session = Object.keys(this.data)[0]
        const datasetT = {
          label: 'Temperature',
          xAxisID: 'time-axis',
          yAxisID: 'temperature-axis',
          showLine: true,
          data: [],
          fill: false,
          borderColor: '#ff5a5f',
          pointRadius: 0
        }
        const datasetTS = {
          label: 'Temperature Setpoint',
          xAxisID: 'time-axis',
          yAxisID: 'temperature-axis',
          showLine: true,
          data: [],
          fill: false,
          borderColor: '#ff5a5f',
          pointRadius: 0,
          borderDash: [5,5]
        }
        const datasetTD = {
          label: 'Temperature Duty',
          xAxisID: 'time-axis',
          yAxisID: 'temperature-axis',
          showLine: true,
          borderWidth: 1,
          data: [],
          fill: true,
          borderColor: '#ff5a5f', //#eabe7c
          backgroundColor: "rgba(255, 90, 95, 0.1)", //'#ff5a5f'
          pointRadius: 0
        }
        const datasetP = {
          label: 'Pressure',
          xAxisID: 'time-axis',
          yAxisID: 'pressure-axis',
          showLine: true,
          data: [],
          fill: false,
          borderColor: '#769fb6',
          pointRadius: 0
        }
        const datasetPS = {
          label: 'Pressure Setpoint',
          xAxisID: 'time-axis',
          yAxisID: 'pressure-axis',
          showLine: true,
          data: [],
          fill: false,
          borderColor: '#769fb6',
          pointRadius: 0,
          borderDash: [5,5]
        }
        const datasetPD = {
          label: 'Pressure Duty',
          xAxisID: 'time-axis',
          yAxisID: 'pressure-axis',
          showLine: true,
          borderWidth: 1,
          data: [],
          fill: true,
          borderColor: '#769fb6', //#eabe7c
          backgroundColor: "rgba(118, 159, 182, 0.1)",
          pointRadius: 0
        }
        const datasetM = {
          label: 'Extraction',
          xAxisID: 'time-axis',
          yAxisID: 'mass-axis',
          showLine: true,
          data: [],
          fill: false,
          borderColor: '#7fd1b9'
        }

        // Loop through responses and add data
        const xPoint0 = new Date(this.data[session][0].t)
        let lastPointWasBrewing = false  // Use to determine when brewing ends to break loop

        this.data[session].every((responseItem, responseIndex) => {
          if ((responseItem.brew && this.graphBrewOnly) || !this.graphBrewOnly) {
            const xPoint = (new Date(responseItem.t) - xPoint0) / 1000
            datasetT.data.push({
              x: xPoint,
              y: responseItem.temperature
            })
            datasetTS.data.push({
              x: xPoint,
              y: responseItem.temperature_setpoint
            })
            datasetTD.data.push({
              x: xPoint,
              y: responseItem.heater_duty
            })
            datasetP.data.push({
              x: xPoint,
              y: responseItem.pressure / 100000  // Convert to bar
            })
            datasetPS.data.push({
              x: xPoint,
              y: responseItem.pressure_setpoint / 100000  // Convert to bar
            })
            datasetPD.data.push({
              x: xPoint,
              y: responseItem.pressure_duty
            })
            datasetM.data.push({
              x: xPoint,
              y: responseItem.m
            })
          }  // endif
          if (this.graphBrewOnly && (!responseItem.brew && lastPointWasBrewing)) { return false }  // break if brewing is stopped
          else {
            lastPointWasBrewing = responseItem.brew
            return true
          }
        })
        graphData.datasets.push(datasetT)
        graphData.datasets.push(datasetTS)
        graphData.datasets.push(datasetTD)
        graphData.datasets.push(datasetP)
        graphData.datasets.push(datasetPS)
        graphData.datasets.push(datasetPD)
        graphData.datasets.push(datasetM)
      }
      return graphData
    },
    tableData: function () {
      const session = Object.keys(this.data)[0]
      const tableData = []
      const t0 = new Date(this.data[session][0].t)
      this.data[session].forEach((responseItem, responseIndex) => {
        tableData.push({
          ts: responseItem.t,
          t: (new Date(responseItem.t) - t0) / 1000,
          heater_duty: responseItem.heater_duty,
          temperature: responseItem.temperature
        })
      })
      return tableData
    }
  },
  methods: {
    showData: function () {
      this.hideData = false
    },
    exportData: function () {
      // window.localStorage.setItem('data', JSON.stringify(this.tableData))
      const data = JSON.stringify(this.tableData)
      const blob = new Blob([data], { type: 'text/plain' })
      const e = document.createEvent('MouseEvents')
      const a = document.createElement('a')
      a.download = 'data.json'
      a.href = window.URL.createObjectURL(blob)
      a.dataset.downloadurl = ['text/json', a.download, a.href].join(':')
      e.initEvent('click', true, false, window, 0, 0, 0, 0, 0, false, false, false, false, 0, null)
      a.dispatchEvent(e)
    }
  },

  created () {
    window.addEventListener ('resize', () => {
      this.chartWidth = 0.8 * window.innerWidth
    })
  }
}
</script>

<style scoped>

</style>
