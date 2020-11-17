(window.webpackJsonp=window.webpackJsonp||[]).push([[23],{372:function(e,t,r){"use strict";r.r(t);var o=r(42),a=Object(o.a)({},(function(){var e=this,t=e.$createElement,r=e._self._c||t;return r("ContentSlotsDistributor",{attrs:{"slot-key":e.$parent.slotKey}},[r("h1",{attrs:{id:"scale-setup"}},[r("a",{staticClass:"header-anchor",attrs:{href:"#scale-setup"}},[e._v("#")]),e._v(" Scale Setup")]),e._v(" "),r("p",[e._v("The scale uses the NodeMCU board with "),r("a",{attrs:{href:"https://github.com/esp8266/Arduino",target:"_blank",rel:"noopener noreferrer"}},[e._v("ESP8266 Arduino Core library"),r("OutboundLink")],1),e._v(".")]),e._v(" "),r("p",[e._v("The Arduino libraries required are:")]),e._v(" "),r("ul",[r("li",[r("a",{attrs:{href:"https://github.com/esp8266/Arduino",target:"_blank",rel:"noopener noreferrer"}},[e._v("ESP8266 Arduino Core library"),r("OutboundLink")],1)]),e._v(" "),r("li",[r("a",{attrs:{href:"https://github.com/pasko-zh/brzo_i2c",target:"_blank",rel:"noopener noreferrer"}},[e._v("Brzo I2C"),r("OutboundLink")],1)]),e._v(" "),r("li",[r("a",{attrs:{href:"https://github.com/ThingPulse/esp8266-oled-ssd1306",target:"_blank",rel:"noopener noreferrer"}},[e._v("ESP8266 and ESP32 OLED driver for SSD1306 displays"),r("OutboundLink")],1)])]),e._v(" "),r("p",[e._v("To use the code you will first need to create a "),r("code",[e._v("wifi_details.h")]),e._v(" file in the /silvia_scale directory with the following  contents, replacing with your WiFi SSID and password:")]),e._v(" "),r("div",{staticClass:"language-cpp extra-class"},[r("pre",{pre:!0,attrs:{class:"language-cpp"}},[r("code",[r("span",{pre:!0,attrs:{class:"token macro property"}},[e._v("#"),r("span",{pre:!0,attrs:{class:"token directive keyword"}},[e._v("ifndef")]),e._v(" WIFI_DETAILS_H")]),e._v("\n"),r("span",{pre:!0,attrs:{class:"token macro property"}},[e._v("#"),r("span",{pre:!0,attrs:{class:"token directive keyword"}},[e._v("define")]),e._v(" WIFI_DETAILS_H")]),e._v("\n"),r("span",{pre:!0,attrs:{class:"token macro property"}},[e._v("#"),r("span",{pre:!0,attrs:{class:"token directive keyword"}},[e._v("define")]),e._v(' WIFI_SSID "********"')]),e._v("\n"),r("span",{pre:!0,attrs:{class:"token macro property"}},[e._v("#"),r("span",{pre:!0,attrs:{class:"token directive keyword"}},[e._v("define")]),e._v(' WIFI_PASSWORD "********"')]),e._v("\n"),r("span",{pre:!0,attrs:{class:"token macro property"}},[e._v("#"),r("span",{pre:!0,attrs:{class:"token directive keyword"}},[e._v("endif")]),e._v(" ")]),r("span",{pre:!0,attrs:{class:"token comment"}},[e._v("// WIFI_DETAILS_H")]),e._v("\n")])])]),r("p",[e._v("You will need to install the board manager as described on the "),r("a",{attrs:{href:"https://github.com/esp8266/Arduino#installing-with-boards-manager",target:"_blank",rel:"noopener noreferrer"}},[e._v("Github page"),r("OutboundLink")],1),e._v(".")]),e._v(" "),r("p",[e._v("Useful information about the board can be found here: "),r("a",{attrs:{href:"https://oneguyoneblog.com/2018/12/28/wemos-d1-esp-wroom-02-arduino-ide/",target:"_blank",rel:"noopener noreferrer"}},[e._v("/oneguyoneblog.com"),r("OutboundLink")],1)]),e._v(" "),r("p",[e._v('To flash the Arduino select "WeMos D1 R1" as the board in the IDE board manager.')]),e._v(" "),r("h2",{attrs:{id:"notes-on-pins"}},[r("a",{staticClass:"header-anchor",attrs:{href:"#notes-on-pins"}},[e._v("#")]),e._v(" Notes on pins")]),e._v(" "),r("p",[e._v("I had real trouble getting the HX711 library to work due to the selection of available pins on the board. Using the D8 (GPIO 15) worked but if connected the board would not start up. Most combinations of pins tried caused the board to constantly reset during the "),r("code",[e._v("setup()")]),e._v(" stage. Int he end D4 (GPIO 2) which is connected to an on board LED and D9 (GPIO 3) worked.")])])}),[],!1,null,null,null);t.default=a.exports}}]);