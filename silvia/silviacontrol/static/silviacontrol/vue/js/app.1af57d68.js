(function(e){function t(t){for(var n,s,o=t[0],c=t[1],l=t[2],u=0,d=[];u<o.length;u++)s=o[u],Object.prototype.hasOwnProperty.call(r,s)&&r[s]&&d.push(r[s][0]),r[s]=0;for(n in c)Object.prototype.hasOwnProperty.call(c,n)&&(e[n]=c[n]);p&&p(t);while(d.length)d.shift()();return i.push.apply(i,l||[]),a()}function a(){for(var e,t=0;t<i.length;t++){for(var a=i[t],n=!0,s=1;s<a.length;s++){var o=a[s];0!==r[o]&&(n=!1)}n&&(i.splice(t--,1),e=c(c.s=a[0]))}return e}var n={},s={app:0},r={app:0},i=[];function o(e){return c.p+"js/"+({about:"about"}[e]||e)+"."+{about:"14b7599c","chunk-2d0b257b":"bcfc8bab","chunk-2d0b3289":"2467fdde","chunk-2d230643":"0915bede","chunk-6c2d7f3e":"3b11a327","chunk-747c0b3c":"3db42798","chunk-c99190f0":"d1c5718b"}[e]+".js"}function c(t){if(n[t])return n[t].exports;var a=n[t]={i:t,l:!1,exports:{}};return e[t].call(a.exports,a,a.exports,c),a.l=!0,a.exports}c.e=function(e){var t=[],a={about:1,"chunk-6c2d7f3e":1,"chunk-c99190f0":1};s[e]?t.push(s[e]):0!==s[e]&&a[e]&&t.push(s[e]=new Promise((function(t,a){for(var n="css/"+({about:"about"}[e]||e)+"."+{about:"1f2c2920","chunk-2d0b257b":"31d6cfe0","chunk-2d0b3289":"31d6cfe0","chunk-2d230643":"31d6cfe0","chunk-6c2d7f3e":"ff357ad1","chunk-747c0b3c":"31d6cfe0","chunk-c99190f0":"076c758d"}[e]+".css",r=c.p+n,i=document.getElementsByTagName("link"),o=0;o<i.length;o++){var l=i[o],u=l.getAttribute("data-href")||l.getAttribute("href");if("stylesheet"===l.rel&&(u===n||u===r))return t()}var d=document.getElementsByTagName("style");for(o=0;o<d.length;o++){l=d[o],u=l.getAttribute("data-href");if(u===n||u===r)return t()}var p=document.createElement("link");p.rel="stylesheet",p.type="text/css",p.onload=t,p.onerror=function(t){var n=t&&t.target&&t.target.src||r,i=new Error("Loading CSS chunk "+e+" failed.\n("+n+")");i.code="CSS_CHUNK_LOAD_FAILED",i.request=n,delete s[e],p.parentNode.removeChild(p),a(i)},p.href=r;var m=document.getElementsByTagName("head")[0];m.appendChild(p)})).then((function(){s[e]=0})));var n=r[e];if(0!==n)if(n)t.push(n[2]);else{var i=new Promise((function(t,a){n=r[e]=[t,a]}));t.push(n[2]=i);var l,u=document.createElement("script");u.charset="utf-8",u.timeout=120,c.nc&&u.setAttribute("nonce",c.nc),u.src=o(e);var d=new Error;l=function(t){u.onerror=u.onload=null,clearTimeout(p);var a=r[e];if(0!==a){if(a){var n=t&&("load"===t.type?"missing":t.type),s=t&&t.target&&t.target.src;d.message="Loading chunk "+e+" failed.\n("+n+": "+s+")",d.name="ChunkLoadError",d.type=n,d.request=s,a[1](d)}r[e]=void 0}};var p=setTimeout((function(){l({type:"timeout",target:u})}),12e4);u.onerror=u.onload=l,document.head.appendChild(u)}return Promise.all(t)},c.m=e,c.c=n,c.d=function(e,t,a){c.o(e,t)||Object.defineProperty(e,t,{enumerable:!0,get:a})},c.r=function(e){"undefined"!==typeof Symbol&&Symbol.toStringTag&&Object.defineProperty(e,Symbol.toStringTag,{value:"Module"}),Object.defineProperty(e,"__esModule",{value:!0})},c.t=function(e,t){if(1&t&&(e=c(e)),8&t)return e;if(4&t&&"object"===typeof e&&e&&e.__esModule)return e;var a=Object.create(null);if(c.r(a),Object.defineProperty(a,"default",{enumerable:!0,value:e}),2&t&&"string"!=typeof e)for(var n in e)c.d(a,n,function(t){return e[t]}.bind(null,n));return a},c.n=function(e){var t=e&&e.__esModule?function(){return e["default"]}:function(){return e};return c.d(t,"a",t),t},c.o=function(e,t){return Object.prototype.hasOwnProperty.call(e,t)},c.p="/",c.oe=function(e){throw console.error(e),e};var l=window["webpackJsonp"]=window["webpackJsonp"]||[],u=l.push.bind(l);l.push=t,l=l.slice();for(var d=0;d<l.length;d++)t(l[d]);var p=u;i.push([0,"chunk-vendors"]),a()})({0:function(e,t,a){e.exports=a("56d7")},"16aa":function(e,t,a){"use strict";var n,s,r=a("1fca"),i=r["b"].reactiveProp,o={name:"ResponseChart",extends:r["a"],mixins:[i],data:function(){return{chartDataLocal:{datasets:[]},maxLines:5}},props:{chartOptions:{type:Object,default:null}},mounted:function(){this.renderChart(this.chartData,this.chartOptions)}},c=o,l=a("2877"),u=Object(l["a"])(c,n,s,!1,null,null,null);t["a"]=u.exports},"30eb":function(e,t,a){"use strict";var n=a("7ba5"),s=a.n(n);s.a},"44ea":function(e,t,a){"use strict";var n=function(){var e=this,t=e.$createElement,a=e._self._c||t;return a("div",[a("v-card",{staticClass:"mb-4 mx-2",staticStyle:{position:"relative"},attrs:{width:e.chartWidth,"min-width":"350","max-width":"1000"}},[a("response-chart",{staticClass:"px-6 pt-6 pb-1",attrs:{chartData:e.graphData,chartOptions:e.graphOptions}}),a("v-card-actions",[a("v-spacer"),a("v-btn",{attrs:{color:"secondary"},on:{click:function(t){e.graphBrewOnly=!e.graphBrewOnly}}},[e._v("isolate brew")]),a("v-btn",{attrs:{color:"info"},on:{click:function(t){e.dataHidden=!e.dataHidden}}},[e._v("data")])],1)],1),e.dataHidden?e._e():a("v-card",{staticClass:"mx-auto mb-4",attrs:{"min-width":"750","max-width":"1600"}},[a("v-data-table",{staticClass:"mx-4",attrs:{headers:e.headers,items:e.tableData,dense:""}}),a("v-card-actions",[a("v-spacer"),a("v-btn",{attrs:{color:"secondary"},on:{click:e.exportData}},[e._v("export")])],1)],1)],1)},s=[],r=(a("a623"),a("4160"),a("a15b"),a("b64b"),a("d3b7"),a("3ca3"),a("159b"),a("ddb0"),a("2b3d"),a("16aa")),i={name:"SingleResponseChart",components:{ResponseChart:r["a"]},data:function(){return{graphBrewOnly:!1,chartWidth:.8*window.innerWidth,graphOptions:{scales:{xAxes:[{id:"time-axis",position:"bottom",display:!0,scaleLabel:{display:!0,labelString:"Time [s]"},ticks:{beginAtZero:!0}}],yAxes:[{id:"temperature-axis",position:"left",display:!0,scaleLabel:{display:!0,labelString:["Temperature [C]","Duty [%]"]},ticks:{beginAtZero:!0,suggestedMin:0,suggestedMax:110,fontColor:"#ff5a5f"}},{id:"mass-axis",position:"right",display:!0,scaleLabel:{display:!0,labelString:"Mass (g)"},ticks:{suggestedMin:0,suggestedMax:30,fontColor:"#7fd1b9"}},{id:"pressure-axis",position:"right",display:!0,scaleLabel:{display:!0,labelString:"Pressure (bar)"},ticks:{suggestedMin:0,suggestedMax:10,fontColor:"#769fb6"}}]},showLines:!0,maintainAspectRatio:!1,responsive:!0},dataHidden:!0,headers:[{text:"Time Stamp",value:"ts"},{text:"Time",value:"t"},{text:"Duty",value:"heater_duty"},{text:"Temperature",value:"temperature"}]}},props:{data:{},brewing:Boolean},computed:{graphData:function(){var e=this,t={};if(null!=this.data){t.datasets=[];var a=Object.keys(this.data)[0],n={label:"Temperature",xAxisID:"time-axis",yAxisID:"temperature-axis",showLine:!0,data:[],fill:!1,borderColor:"#ff5a5f",pointRadius:0},s={label:"Temperature Setpoint",xAxisID:"time-axis",yAxisID:"temperature-axis",showLine:!0,data:[],fill:!1,borderColor:"#ff5a5f",pointRadius:0,borderDash:[5,5]},r={label:"Temperature Duty",xAxisID:"time-axis",yAxisID:"temperature-axis",showLine:!0,borderWidth:1,data:[],fill:!0,borderColor:"#ff5a5f",backgroundColor:"rgba(255, 90, 95, 0.1)",pointRadius:0},i={label:"Pressure",xAxisID:"time-axis",yAxisID:"pressure-axis",showLine:!0,data:[],fill:!1,borderColor:"#769fb6",pointRadius:0},o={label:"Pressure Setpoint",xAxisID:"time-axis",yAxisID:"pressure-axis",showLine:!0,data:[],fill:!1,borderColor:"#769fb6",pointRadius:0,borderDash:[5,5]},c={label:"Pressure Duty",xAxisID:"time-axis",yAxisID:"pressure-axis",showLine:!0,borderWidth:1,data:[],fill:!0,borderColor:"#769fb6",backgroundColor:"rgba(118, 159, 182, 0.1)",pointRadius:0},l={label:"Extraction",xAxisID:"time-axis",yAxisID:"mass-axis",showLine:!0,data:[],fill:!1,borderColor:"#7fd1b9"},u=new Date(this.data[a][0].t),d=!1;this.data[a].every((function(t,a){if(t.brew&&e.graphBrewOnly||!e.graphBrewOnly){var p=(new Date(t.t)-u)/1e3;n.data.push({x:p,y:t.temperature}),s.data.push({x:p,y:t.temperature_setpoint}),r.data.push({x:p,y:t.heater_duty}),i.data.push({x:p,y:t.pressure/1e5}),o.data.push({x:p,y:t.pressure_setpoint/1e5}),c.data.push({x:p,y:t.pressure_duty}),l.data.push({x:p,y:t.m})}return!(e.graphBrewOnly&&!t.brew&&d)&&(d=t.brew,!0)})),t.datasets.push(n),t.datasets.push(s),t.datasets.push(r),t.datasets.push(i),t.datasets.push(o),t.datasets.push(c),t.datasets.push(l)}return t},tableData:function(){var e=Object.keys(this.data)[0],t=[],a=new Date(this.data[e][0].t);return this.data[e].forEach((function(e,n){t.push({ts:e.t,t:(new Date(e.t)-a)/1e3,heater_duty:e.heater_duty,temperature:e.temperature})})),t}},methods:{showData:function(){this.hideData=!1},exportData:function(){var e=JSON.stringify(this.tableData),t=new Blob([e],{type:"text/plain"}),a=document.createEvent("MouseEvents"),n=document.createElement("a");n.download="data.json",n.href=window.URL.createObjectURL(t),n.dataset.downloadurl=["text/json",n.download,n.href].join(":"),a.initEvent("click",!0,!1,window,0,0,0,0,0,!1,!1,!1,!1,0,null),n.dispatchEvent(a)}},created:function(){var e=this;window.addEventListener("resize",(function(){e.chartWidth=.8*window.innerWidth}))}},o=i,c=a("2877"),l=a("6544"),u=a.n(l),d=a("8336"),p=a("b0af"),m=a("99d9"),f=a("8fea"),h=a("2fa4"),b=Object(c["a"])(o,n,s,!1,null,"6fadedf8",null);t["a"]=b.exports;u()(b,{VBtn:d["a"],VCard:p["a"],VCardActions:m["a"],VDataTable:f["a"],VSpacer:h["a"]})},4678:function(e,t,a){var n={"./af":"2bfb","./af.js":"2bfb","./ar":"8e73","./ar-dz":"a356","./ar-dz.js":"a356","./ar-kw":"423e","./ar-kw.js":"423e","./ar-ly":"1cfd","./ar-ly.js":"1cfd","./ar-ma":"0a84","./ar-ma.js":"0a84","./ar-sa":"8230","./ar-sa.js":"8230","./ar-tn":"6d83","./ar-tn.js":"6d83","./ar.js":"8e73","./az":"485c","./az.js":"485c","./be":"1fc1","./be.js":"1fc1","./bg":"84aa","./bg.js":"84aa","./bm":"a7fa","./bm.js":"a7fa","./bn":"9043","./bn.js":"9043","./bo":"d26a","./bo.js":"d26a","./br":"6887","./br.js":"6887","./bs":"2554","./bs.js":"2554","./ca":"d716","./ca.js":"d716","./cs":"3c0d","./cs.js":"3c0d","./cv":"03ec","./cv.js":"03ec","./cy":"9797","./cy.js":"9797","./da":"0f14","./da.js":"0f14","./de":"b469","./de-at":"b3eb","./de-at.js":"b3eb","./de-ch":"bb71","./de-ch.js":"bb71","./de.js":"b469","./dv":"598a","./dv.js":"598a","./el":"8d47","./el.js":"8d47","./en-au":"0e6b","./en-au.js":"0e6b","./en-ca":"3886","./en-ca.js":"3886","./en-gb":"39a6","./en-gb.js":"39a6","./en-ie":"e1d3","./en-ie.js":"e1d3","./en-il":"7333","./en-il.js":"7333","./en-in":"ec2e","./en-in.js":"ec2e","./en-nz":"6f50","./en-nz.js":"6f50","./en-sg":"b7e9","./en-sg.js":"b7e9","./eo":"65db","./eo.js":"65db","./es":"898b","./es-do":"0a3c","./es-do.js":"0a3c","./es-us":"55c9","./es-us.js":"55c9","./es.js":"898b","./et":"ec18","./et.js":"ec18","./eu":"0ff2","./eu.js":"0ff2","./fa":"8df4","./fa.js":"8df4","./fi":"81e9","./fi.js":"81e9","./fil":"d69a","./fil.js":"d69a","./fo":"0721","./fo.js":"0721","./fr":"9f26","./fr-ca":"d9f8","./fr-ca.js":"d9f8","./fr-ch":"0e49","./fr-ch.js":"0e49","./fr.js":"9f26","./fy":"7118","./fy.js":"7118","./ga":"5120","./ga.js":"5120","./gd":"f6b4","./gd.js":"f6b4","./gl":"8840","./gl.js":"8840","./gom-deva":"aaf2","./gom-deva.js":"aaf2","./gom-latn":"0caa","./gom-latn.js":"0caa","./gu":"e0c5","./gu.js":"e0c5","./he":"c7aa","./he.js":"c7aa","./hi":"dc4d","./hi.js":"dc4d","./hr":"4ba9","./hr.js":"4ba9","./hu":"5b14","./hu.js":"5b14","./hy-am":"d6b6","./hy-am.js":"d6b6","./id":"5038","./id.js":"5038","./is":"0558","./is.js":"0558","./it":"6e98","./it-ch":"6f12","./it-ch.js":"6f12","./it.js":"6e98","./ja":"079e","./ja.js":"079e","./jv":"b540","./jv.js":"b540","./ka":"201b","./ka.js":"201b","./kk":"6d79","./kk.js":"6d79","./km":"e81d","./km.js":"e81d","./kn":"3e92","./kn.js":"3e92","./ko":"22f8","./ko.js":"22f8","./ku":"2421","./ku.js":"2421","./ky":"9609","./ky.js":"9609","./lb":"440c","./lb.js":"440c","./lo":"b29d","./lo.js":"b29d","./lt":"26f9","./lt.js":"26f9","./lv":"b97c","./lv.js":"b97c","./me":"293c","./me.js":"293c","./mi":"688b","./mi.js":"688b","./mk":"6909","./mk.js":"6909","./ml":"02fb","./ml.js":"02fb","./mn":"958b","./mn.js":"958b","./mr":"39bd","./mr.js":"39bd","./ms":"ebe4","./ms-my":"6403","./ms-my.js":"6403","./ms.js":"ebe4","./mt":"1b45","./mt.js":"1b45","./my":"8689","./my.js":"8689","./nb":"6ce3","./nb.js":"6ce3","./ne":"3a39","./ne.js":"3a39","./nl":"facd","./nl-be":"db29","./nl-be.js":"db29","./nl.js":"facd","./nn":"b84c","./nn.js":"b84c","./oc-lnc":"167b","./oc-lnc.js":"167b","./pa-in":"f3ff","./pa-in.js":"f3ff","./pl":"8d57","./pl.js":"8d57","./pt":"f260","./pt-br":"d2d4","./pt-br.js":"d2d4","./pt.js":"f260","./ro":"972c","./ro.js":"972c","./ru":"957c","./ru.js":"957c","./sd":"6784","./sd.js":"6784","./se":"ffff","./se.js":"ffff","./si":"eda5","./si.js":"eda5","./sk":"7be6","./sk.js":"7be6","./sl":"8155","./sl.js":"8155","./sq":"c8f3","./sq.js":"c8f3","./sr":"cf1e","./sr-cyrl":"13e9","./sr-cyrl.js":"13e9","./sr.js":"cf1e","./ss":"52bd","./ss.js":"52bd","./sv":"5fbd","./sv.js":"5fbd","./sw":"74dc","./sw.js":"74dc","./ta":"3de5","./ta.js":"3de5","./te":"5cbb","./te.js":"5cbb","./tet":"576c","./tet.js":"576c","./tg":"3b1b","./tg.js":"3b1b","./th":"10e8","./th.js":"10e8","./tl-ph":"0f38","./tl-ph.js":"0f38","./tlh":"cf75","./tlh.js":"cf75","./tr":"0e81","./tr.js":"0e81","./tzl":"cf51","./tzl.js":"cf51","./tzm":"c109","./tzm-latn":"b53d","./tzm-latn.js":"b53d","./tzm.js":"c109","./ug-cn":"6117","./ug-cn.js":"6117","./uk":"ada2","./uk.js":"ada2","./ur":"5294","./ur.js":"5294","./uz":"2e8c","./uz-latn":"010e","./uz-latn.js":"010e","./uz.js":"2e8c","./vi":"2921","./vi.js":"2921","./x-pseudo":"fd7e","./x-pseudo.js":"fd7e","./yo":"7f33","./yo.js":"7f33","./zh-cn":"5c3a","./zh-cn.js":"5c3a","./zh-hk":"49ab","./zh-hk.js":"49ab","./zh-mo":"3a6c","./zh-mo.js":"3a6c","./zh-tw":"90ea","./zh-tw.js":"90ea"};function s(e){var t=r(e);return a(t)}function r(e){if(!a.o(n,e)){var t=new Error("Cannot find module '"+e+"'");throw t.code="MODULE_NOT_FOUND",t}return n[e]}s.keys=function(){return Object.keys(n)},s.resolve=r,e.exports=s,s.id="4678"},5345:function(e,t,a){e.exports=a.p+"img/Silvia_Illustration_on.75f5c1e9.png"},"56d7":function(e,t,a){"use strict";a.r(t),a.d(t,"eventBus",(function(){return ze}));a("4de4"),a("e260"),a("e6cf"),a("cca6"),a("a79d");var n=a("2b0e"),s=function(){var e=this,t=e.$createElement,a=e._self._c||t;return a("v-app",[a("AppNaviagtion",{attrs:{machineOn:e.machineOn}}),a("v-content",[a("v-container",{staticClass:"mt-5",attrs:{fluid:""}},[a("v-row",{attrs:{align:"center",justify:"center"}},[a("router-view",{attrs:{machineOn:e.machineOn,machineBrewing:e.machineBrewing,machineMode:e.machineMode}})],1)],1)],1),a("v-footer",{attrs:{app:""}})],1)},r=[],i=(a("b0c0"),a("a9e3"),a("bc3a")),o=a.n(i),c=a("e86b"),l=a.n(c),u=new l.a.Ros({url:"undefined"!==typeof webpackHotUpdate?"ws://localhost:9090":"ws://192.168.0.6:9090"});console.log(u),u.on("connection",(function(){console.log("Connected to websocket server.")})),u.on("error",(function(e){console.log("Error connecting to websocket server: ",e)})),u.on("close",(function(){console.log("Connection to websocket server closed.")}));var d=function(){var e=this,t=e.$createElement,n=e._self._c||t;return n("span",[n("v-navigation-drawer",{attrs:{app:"",clipped:!0},model:{value:e.drawer,callback:function(t){e.drawer=t},expression:"drawer"}},[n("v-list",[e._l(e.items,(function(t){return[n("router-link",{key:t.title,attrs:{to:t.path}},[n("v-list-item",{attrs:{link:""}},[n("v-list-item-action",[n("v-icon",[e._v(e._s(t.icon))])],1),n("v-list-item-content",[n("v-list-item-title",[e._v(e._s(t.title))])],1)],1)],1)]}))],2),n("a",{attrs:{href:"admin"}},[n("v-list-item",{staticClass:"primary darken-2",staticStyle:{position:"absolute",bottom:"0",width:"100%"},attrs:{link:""}},[n("v-list-item-action",[n("v-icon",[e._v("mdi-lock")])],1),n("v-list-item-content",[n("v-list-item-title",[e._v("Admin")])],1)],1)],1)],1),n("v-app-bar",{attrs:{app:"",color:"primary","clipped-left":!0}},[n("v-app-bar-nav-icon",{on:{click:function(t){t.stopPropagation(),e.drawer=!e.drawer}}}),n("v-spacer",{staticClass:"hidden-md-and-up"}),n("v-img",{staticClass:"mx-2",attrs:{src:a("cf05"),"max-height":"40","max-width":"40",contain:""}}),n("v-toolbar-title",{staticClass:"hidden-sm-and-down"},[e._v(e._s(e.appTitle))]),n("v-spacer",{staticClass:"hidden-sm-and-down"}),n("v-btn",{staticClass:"hidden-sm-and-down",attrs:{color:"secondary",to:"/"},on:{click:e.toggleOnOff}},[e.machineOn?n("div",[n("v-icon",{attrs:{color:"success"}},[e._v("mdi-power")])],1):n("div",[n("v-icon",[e._v("mdi-power")])],1)])],1)],1)},p=[],m={name:"AppNavigation",data:function(){return{appTitle:"Silvia Control",drawer:!1,items:[{title:"Operate",icon:"mdi-coffee",path:"/"},{title:"Sessions",icon:"mdi-database",path:"/sessions"},{title:"Schedule",icon:"mdi-calendar",path:"/schedule"},{title:"Information",icon:"mdi-information",path:"/info"},{title:"Settings",icon:"mdi-cog",path:"/settings"},{title:"Docs",icon:"mdi-bookshelf",path:"/docs"},{title:"About",icon:"mdi-help-circle",path:"/about"}]}},props:{machineOn:Boolean},methods:{toggleOnOff:function(){this.machineOn?ze.$emit("changeMode",0):ze.$emit("changeMode",1)}}},f=m,h=(a("e2c5"),a("2877")),b=a("6544"),v=a.n(b),g=a("40dc"),j=a("5bc1"),w=a("8336"),y=a("132d"),x=a("adda"),O=a("8860"),A=a("da13"),k=a("1800"),D=a("5d23"),S=a("f774"),_=a("2fa4"),C=a("2a7f"),B=Object(h["a"])(f,d,p,!1,null,"11fa1904",null),N=B.exports;v()(B,{VAppBar:g["a"],VAppBarNavIcon:j["a"],VBtn:w["a"],VIcon:y["a"],VImg:x["a"],VList:O["a"],VListItem:A["a"],VListItemAction:k["a"],VListItemContent:D["a"],VListItemTitle:D["b"],VNavigationDrawer:S["a"],VSpacer:_["a"],VToolbarTitle:C["a"]});var z=new l.a.Topic({ros:u,name:"/status",messageType:"django_interface/SilviaStatus"});o.a.defaults.xsrfCookieName="csrftoken",o.a.defaults.xsrfHeaderName="X-CSRFToken",o.a.defaults.headers["Content-Type"]="application/json",o.a.defaults.withCredentials=!0,o.a.defaults.trailingSlash=!0,o.a.interceptors.request.use((function(e){return e.addTrailingSlash&&"/"!==e.url[e.url.length-1]&&(e.url+="/"),e}));var I={name:"App",components:{AppNaviagtion:N},data:function(){return{machineOn:!1,machineBrewing:!1,machineMode:0,MODE_IGNORE:-1,MODE_OFF:0,MODE_PID:1,MODE_MANUAL:2,MODE_CLEAN:3}},watch:{$route:function(e,t){document.title=e.meta.title||"Silvia"}},created:function(){var e=this;z.subscribe((function(t){console.log("Received message on "+z.name+": "+t.mode),e.machineBrewing=Boolean(t.brew),e.machineMode=Number(t.mode),e.machineOn=t.mode!=e.MODE_OFF})),ze.$on("toggleBrew",(function(){var t={id:1,mode:e.machineMode,brew:!e.machineBrewing};o.a.put("/api/v1/status/1/",t).then((function(e){console.log(e)})).catch((function(e){return console.log(e)}))})),ze.$on("changeMode",(function(t){var a={id:1,brew:e.machineBrewing,mode:t};o.a.put("/api/v1/status/1/",a).then((function(e){console.log(e)})).catch((function(e){return console.log(e)}))})),ze.$on("updateStatus",(function(){o.a.get("/api/v1/status/1/").then((function(t){console.log("Updating status"),e.machineBrewing=Boolean(t.data.brew),e.machineMode=Number(t.data.mode),e.machineOn=t.data.mode!=e.MODE_OFF})).catch((function(e){return console.log(e)}))}))}},M=I,V=a("7496"),T=a("a523"),L=a("a75b"),E=a("553a"),R=a("0fd9"),Z=Object(h["a"])(M,s,r,!1,null,null,null),K=Z.exports;v()(Z,{VApp:V["a"],VContainer:T["a"],VContent:L["a"],VFooter:E["a"],VRow:R["a"]});var H=a("9483");Object(H["a"])("".concat("/","service-worker.js"),{ready:function(){console.log("App is being served from cache by a service worker.\nFor more details, visit https://goo.gl/AFskqB")},registered:function(){console.log("Service worker has been registered.")},cached:function(){console.log("Content has been cached for offline use.")},updatefound:function(){console.log("New content is downloading.")},updated:function(){console.log("New content is available; please refresh.")},offline:function(){console.log("No internet connection found. App is running in offline mode.")},error:function(e){console.error("Error during service worker registration:",e)}});a("d3b7");var P=a("8c4f"),U=function(){var e=this,t=e.$createElement,a=e._self._c||t;return a("div",{staticClass:"home"},[a("MachineInterface",{attrs:{machineOn:e.machineOn,machineBrewing:e.machineBrewing,machineMode:e.machineMode}})],1)},W=[],q=function(){var e=this,t=e.$createElement,a=e._self._c||t;return a("div",{staticClass:"machine-interface"},[a("div",{staticClass:"machine-container"},["machine"==e.displayOption?a("div",[a("MachineDisplay",{attrs:{machineOn:e.machineOn,temperature:e.response.temperature,temperature_setpoint:e.response.temperature_setpoint,pressure:e.response.pressure,mass:e.response.mass,brew_time:e.t_brew}})],1):e._e(),"graph"==e.displayOption?a("div",[0==e.loggedData.current.length?a("div",{staticStyle:{display:"flex","justify-content":"center","align-items":"center","min-width":"350px"}},[a("v-btn",{attrs:{color:"secondary"},on:{click:e.viewLastSession}},[e._v("Last Session")])],1):a("div",[a("single-response-chart",{attrs:{data:e.loggedData}})],1)]):e._e()]),a("br"),a("v-row",{attrs:{align:"center"}},[a("v-col",{attrs:{cols:"auto"}},[a("v-switch",{attrs:{color:"secondary",value:"","input-value":e.machineOn,label:e.machineOn?"On":"Off"},on:{change:e.toggleOnOff}})],1),a("v-spacer"),e.machineOn?a("v-col",{staticClass:"px-1",attrs:{cols:"auto"}},[e.machineBrewing?a("div",[a("v-btn",{attrs:{color:"error",fab:"",small:"",elevation:"1"},on:{click:e.toggleBrew}},[a("v-icon",[e._v("mdi-coffee")])],1)],1):a("div",[a("v-btn",{attrs:{color:"info",fab:"",small:"",elevation:"1"},on:{click:e.toggleBrew}},[a("v-icon",[e._v("mdi-coffee")])],1)],1)]):e._e(),e.machineOn?a("v-col",{staticClass:"px-1",attrs:{cols:"auto"}},[3==e.machineMode?a("div",[a("v-btn",{attrs:{color:"error",fab:"",small:"",elevation:"1"},on:{click:e.toggleClean}},[a("v-icon",[e._v("mdi-dishwasher-off")])],1)],1):a("div",[a("v-btn",{attrs:{color:"secondary",fab:"",small:"",elevation:"1"},on:{click:e.toggleClean}},[a("v-icon",[e._v("mdi-dishwasher")])],1)],1)]):e._e(),a("v-col",{staticClass:"px-1",attrs:{cols:"auto"}},[e.machineOn?a("v-btn",{attrs:{fab:"",small:"",outlined:"",color:e.waterLevelColor}},[a("v-icon",[e._v("mdi-water")])],1):e._e()],1),a("v-col",{staticClass:"px-1",attrs:{cols:"auto"}},[a("v-btn",{attrs:{color:"secondary",fab:"",small:"",elevation:"1",outlined:""},on:{click:e.changeDisplay}},["machine"==e.displayOption?a("div",[a("v-icon",[e._v("mdi-chart-line")])],1):a("div",[a("v-icon",[e._v("mdi-file-presentation-box")])],1)])],1),a("v-col",{staticClass:"px-1",attrs:{cols:"auto"}},[a("v-btn",{attrs:{color:"secondary",fab:"",small:"",elevation:"1",outlined:""},on:{click:e.toggleOverride}},[a("v-icon",[e._v("mdi-wrench")])],1)],1)],1),e.machineBrewing?a("div",[a("v-row",{attrs:{align:"center"}},[a("v-progress-linear",{attrs:{value:e.brewProgress,color:"blue-grey",height:"25",rounded:""}},[null==e.response.mass?a("div",[e._v(" No scale detected ")]):a("div",[e._v(" "+e._s(e._f("temperatureDisplayFilter")(e.response.mass))+"g / "+e._s(e.response.mass_setpoint)+"g ")])])],1)],1):e._e(),2!=e.machineMode||e.machineBrewing?e._e():a("div",[a("v-row",{attrs:{align:"center"}},[a("v-col",{staticClass:"px-1",attrs:{cols:"auto","align-self":"baseline"}},[a("v-text-field",{staticStyle:{"max-width":"70px"},attrs:{type:"number",suffix:"%",label:"Heater duty"},model:{value:e.heaterDuty,callback:function(t){e.heaterDuty=t},expression:"heaterDuty"}})],1),a("v-col",{staticClass:"px-1",attrs:{cols:"auto"}},[a("v-btn",{attrs:{color:"secondary"},on:{click:e.sendHeaterDuty}},[e._v(" Heater ")])],1),a("v-spacer"),a("v-col",{staticClass:"px-1",attrs:{cols:"auto","align-self":"baseline"}},[a("v-text-field",{staticStyle:{"max-width":"70px"},attrs:{type:"number",suffix:"%",label:"Pump duty"},model:{value:e.pumpDuty,callback:function(t){e.pumpDuty=t},expression:"pumpDuty"}})],1),a("v-col",{staticClass:"px-1",attrs:{cols:"auto"}},[a("v-btn",{attrs:{color:"secondary"},on:{click:e.sendPumpDuty}},[e._v(" Pump ")])],1)],1)],1)],1)},F=[],X=(a("25f0"),function(){var e=this,t=e.$createElement,n=e._self._c||t;return n("div",{staticClass:"machine"},[e.machineOn?n("div",[n("v-img",{staticClass:"machine-image img-center",attrs:{"min-width":"150","max-width":"350",src:a("5345"),contain:""}})],1):n("div",[n("v-img",{staticClass:"machine-image img-center",attrs:{"min-width":"150","max-width":"350",src:a("fd6e"),contain:""}})],1),n("v-btn",{attrs:{id:"temp-btn",outlined:"",color:e.tempBtnColor}},[null==e.temperature?n("div",[e._v("-")]):n("div",[e._v(e._s(e._f("temperatureDisplayFilter")(e.temperature))+" ℃")])]),n("v-btn",{attrs:{id:"pressure-btn",outlined:"",color:"secondary"}},[null==e.pressure?n("div",[e._v("-")]):n("div",[e._v(e._s(e._f("temperatureDisplayFilter")(e.pressure))+" bar")])]),e.machineOn?n("v-btn",{attrs:{id:"brew-btn",outlined:"",text:"",color:"secondary"}},[n("v-col",[n("v-row",{staticClass:"pb-1",attrs:{justify:"center"}},[e._v(e._s(e._f("temperatureDisplayFilter")(e.mass))+"g")]),n("v-row",{attrs:{justify:"center"}},[e._v(e._s(e.brew_time)+"s")])],1)],1):e._e()],1)}),Y=[],Q={name:"MachineDisplay",props:{machineOn:Boolean,temperature:Number,temperature_setpoint:Number,pressure:Number,mass:Number,brew_time:Number},computed:{tempBtnColor:function(){return Math.abs(this.temperature_setpoint-this.temperature)<1?"success":"secondary"}}},J=Q,G=(a("30eb"),a("62ad")),$=Object(h["a"])(J,X,Y,!1,null,"987e7fd2",null),ee=$.exports;v()($,{VBtn:w["a"],VCol:G["a"],VImg:x["a"],VRow:R["a"]});var te=a("44ea"),ae={methods:{getSessions:function(){o.a.get("/api/v1/session/").then((function(e){return e.data})).catch((function(e){return console.log(e)}))}}},ne={name:"MachineInterface",mixins:[ae],components:{MachineDisplay:ee,SingleResponseChart:te["a"]},data:function(){return{response:{temperature:0,temperature_setpoint:0,heater_duty:0,pressure:0,pressure_setpoint:0,pump_duty:0,mass:0,mass_setpoint:0,low_water:!1},heaterDuty:100,pumpDuty:100,displayOption:"machine",n_datapoints:500,loggedData:{current:[]},logIntervalReference:null,t_brew:0}},props:{machineOn:Boolean,machineBrewing:Boolean,machineMode:Number},computed:{brewProgress:function(){return 100*this.response.mass/this.response.mass_setpoint},waterLevelColor:function(){return this.response.low_water?"error":"success"}},methods:{changeDisplay:function(){"machine"===this.displayOption?this.displayOption="graph":this.displayOption="machine"},toggleOnOff:function(){var e=0===this.machineMode?1:0;ze.$emit("changeMode",e)},toggleBrew:function(){ze.$emit("toggleBrew")},toggleOverride:function(){var e=2!=this.machineMode?2:1;ze.$emit("changeMode",e)},toggleClean:function(){var e=3!=this.machineMode?3:1;ze.$emit("changeMode",e)},sendHeaterDuty:function(){var e={params:{duty:this.heaterDuty}};o.a.get("/api/v1/heaterduty/",e).then((function(e){})).catch((function(e){return console.log(e)}))},sendPumpDuty:function(){var e={params:{duty:this.pumpDuty}};o.a.get("/api/v1/pumpduty/",e).then((function(e){})).catch((function(e){return console.log(e)}))},viewLastSession:function(){var e=this;o.a.get("/api/v1/session/").then((function(t){var a=t.data[t.data.length-1];e.$router.push({name:"Session",params:{sessionIds:a.id.toString()}})})).catch((function(e){return console.log(e)}))},logAtInterval:function(){var e=Object.assign({},this.response);e.t=new Date,this.loggedData.current.push(e);while(this.loggedData.length>this.n_datapoints)this.loggedData.pop(0)}},created:function(){var e=this;ze.$emit("updateStatus"),this.logIntervalReference=setInterval((function(){e.logAtInterval()}),500);var t=new l.a.Topic({ros:u,name:"/heater_controller",messageType:"django_interface/SilviaController"});t.subscribe((function(t){e.response.temperature=Number(t.input),e.response.temperature_setpoint=Number(t.setpoint),e.response.heater_duty=Number(t.output)}));var a=new l.a.Topic({ros:u,name:"/pump_controller",messageType:"django_interface/SilviaController"});a.subscribe((function(t){e.response.pressure=Number(t.input),e.response.pressure_setpoint=Number(t.setpoint),e.response.pump_duty=Number(t.output),e.response.brew=Boolean(t.active)}));var n=new l.a.Topic({ros:u,name:"/scale",messageType:"django_interface/SilviaMass"});n.subscribe((function(t){e.response.mass=Number(t.input),e.response.mass_setpoint=Number(t.setpoint)}));var s=new l.a.Topic({ros:u,name:"/low_water",messageType:"django_interface/SilviaWater"});s.subscribe((function(t){e.response.low_water=Boolean(t.low_water)}));var r=new l.a.Topic({ros:u,name:"/brew_duration",messageType:"django_interface/SilviaBrewTimer"});r.subscribe((function(t){e.t_brew=Number(t.duration.secs)}))},destroyed:function(){}},se=ne,re=(a("ba35"),a("8e36")),ie=a("b73d"),oe=a("8654"),ce=Object(h["a"])(se,q,F,!1,null,"5f2bdc09",null),le=ce.exports;v()(ce,{VBtn:w["a"],VCol:G["a"],VIcon:y["a"],VProgressLinear:re["a"],VRow:R["a"],VSpacer:_["a"],VSwitch:ie["a"],VTextField:oe["a"]});var ue={name:"Home",components:{MachineInterface:le},props:{machineOn:Boolean,machineBrewing:Boolean,machineMode:Number}},de=ue,pe=Object(h["a"])(de,U,W,!1,null,"331e977e",null),me=pe.exports,fe=function(){var e=this,t=e.$createElement,a=e._self._c||t;return a("div",{staticClass:"mx-auto text-center"},[a("h1",[e._v("(404)")]),a("h2",[e._v("Page not found")]),a("v-icon",[e._v("mdi-emoticon-sad")])],1)},he=[],be={},ve=be,ge=Object(h["a"])(ve,fe,he,!1,null,null,null),je=ge.exports;v()(ge,{VIcon:y["a"]});var we=function(){return a.e("chunk-6c2d7f3e").then(a.bind(null,"78b4"))},ye=function(){return a.e("chunk-747c0b3c").then(a.bind(null,"3a39f"))},xe=function(){return a.e("chunk-2d0b257b").then(a.bind(null,"2469"))},Oe=function(){return a.e("chunk-2d0b3289").then(a.bind(null,"26d3"))},Ae=function(){return a.e("chunk-c99190f0").then(a.bind(null,"6b7b"))},ke=function(){return a.e("chunk-2d230643").then(a.bind(null,"eba1"))};n["a"].use(P["a"]);var De=[{path:"/",name:"Home",component:me,meta:{title:"Silvia"}},{path:"/session/:sessionIds",name:"Session",component:we,meta:{title:"Session"},props:!0},{path:"/sessions",name:"Sessions",component:ye,meta:{title:"Sessions"}},{path:"/info",name:"Info",component:xe,meta:{title:"Info"}},{path:"/settings",name:"Settings",component:Oe,meta:{title:"Settings"}},{path:"/schedule",name:"Schedule",component:Ae,meta:{title:"Schedule"}},{path:"/about",name:"About",component:function(){return a.e("about").then(a.bind(null,"f820"))},meta:{title:"About Silvia"}},{path:"/docs",name:"Docs",component:ke,meta:{title:"Documentation"}},{path:"*",component:je}],Se=new P["a"]({routes:De}),_e=Se,Ce=a("f309");n["a"].use(Ce["a"]);var Be=new Ce["a"]({theme:{themes:{light:{primary:"#eeeeee",secondary:"#90a4ae",accent:"#ff5722",error:"#f44336",warning:"#e91e63",info:"#9c27b0",success:"#4caf50"}}}}),Ne=a("6612");n["a"].config.productionTip=!1;var ze=new n["a"];n["a"].filter("temperatureDisplayFilter",(function(e){return Ne(e).format("0.0")})),new n["a"]({router:_e,vuetify:Be,render:function(e){return e(K)}}).$mount("#app")},"7ba5":function(e,t,a){},"900b8":function(e,t,a){},b651:function(e,t,a){},ba35:function(e,t,a){"use strict";var n=a("b651"),s=a.n(n);s.a},cf05:function(e,t){e.exports="data:image/png;base64,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"},e2c5:function(e,t,a){"use strict";var n=a("900b8"),s=a.n(n);s.a},fd6e:function(e,t,a){e.exports=a.p+"img/Silvia_Illustration_off.1783be3b.png"}});
//# sourceMappingURL=app.1af57d68.js.map