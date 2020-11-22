<template>
  <div class="machine">

    <div v-if="machineOn">
      <v-img class="machine-image img-center" min-width="150" max-width="350" src="@/assets/Silvia_Illustration_on.png" contain />
    </div>
    <div v-else>
      <v-img class="machine-image img-center" min-width="150" max-width="350" src="@/assets/Silvia_Illustration_off.png" contain />
    </div>
    <v-btn id="temp-btn" outlined :color="tempBtnColor">
      <div v-if="temperature == null">-</div>
      <div v-else>{{ temperature | temperatureDisplayFilter }} &#8451;</div>
    </v-btn>
    <v-btn id="pressure-btn" outlined color="secondary">
      <div v-if="pressure == null">-</div>
      <div v-else>{{ pressure | temperatureDisplayFilter }} bar</div>
    </v-btn>
    <v-btn v-if="machineOn" id="brew-btn" class="" outlined text color="secondary">
        <v-col>
          <v-row class="pb-1" justify="center">{{ mass | temperatureDisplayFilter }}g</v-row>
          <v-row class="" justify="center">{{ brew_time }}s</v-row>
        </v-col>
      </v-btn>
  </div>
</template>

<script>
export default {
  name: 'MachineDisplay',
  props: {
    machineOn: Boolean,
    temperature: Number,
    temperature_setpoint: Number,
    pressure: Number,
    mass: Number,
    brew_time: Number
  },
  computed: {
    tempBtnColor: function () {
      if (Math.abs(this.temperature_setpoint - this.temperature) < 1) {
        return 'success'
      }
      return 'secondary'
    }
  }
}
</script>

<!-- Add "scoped" attribute to limit CSS to this component only -->
<style scoped>
.machine-interface {
  margin: auto;
}
/* Image of machine */
.img-center  {
  display: block;
  margin-left: auto;
  margin-right: auto;
}
/* .machine-image {
    height: 500px;
} */

#temp-btn {
  position: absolute;
  top: 24%;
  left: 50%;
  transform: translate(-50%, -50%);
  -ms-transform: translate(-50%, -50%);
  background-color: rgb(236, 236, 236);
}
#pressure-btn {
  position: absolute;
  top: 33%;
  left: 50%;
  transform: translate(-50%, -50%);
  -ms-transform: translate(-50%, -50%);
  background-color: rgb(236, 236, 236);
  text-transform: none;
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
