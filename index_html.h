const char index_html[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html>
<head>
<title>Plot Device</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="icon" href="data:,">

<style>

html {
  font-family: Arial, Helvetica, sans-serif;
  text-align: center;
}
h1 {
  font-size: 1.8rem;
  color: white;
}
h2{
  font-size: 1.5rem;
  font-weight: bold;
  color: #BABABA;
line-height: .1;
}

h3{
  font-size: 1rem;
  font-weight: bold;
  color: #828282;
line-height: .8
}

h4{
  font-size: .05rem;
  color:#150501;
  line-height: .1
}
.topnav {
  overflow: hidden;
  background-color: #2a0a03 ;
}
body {
  margin: 0;
background-color:#2a0a03;
}
.content {
  padding: 30px;
  max-width: 600px;
  margin: 0 auto;
}
.card {
  background-color: #150501;;
  box-shadow: 2px 2px 12px 1px rgba(140,140,140,.5);
  padding-top:10px;
  padding-bottom:20px;
color: #fff;
}
.button {
  padding: 10px 50px;
  font-size: 24px;
  text-align: center;
  outline: none;
  color: #fff;
  background-color: #383838;
  border: none;
  border-radius: 5px;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  -khtml-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
  -webkit-tap-highlight-color: rgba(0,0,0,0);
 }
 /*.button:hover {background-color: #0f8b8d}*/
 .button:active {
   background-color: #458a8a;
   box-shadow: 2 2px #CDCDCD;
   transform: translateY(2px);
 }

.button2 {
  padding: 5px 20px;
  font-size: 36px;
  text-align: center;
  outline: none;
  color: #fff;
  background-color: #383838;
  border: none;
  border-radius: 10px;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  -khtml-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
  -webkit-tap-highlight-color: rgba(0,0,0,0);
 }
 /*.button2:hover {background-color: #0f8b8d}*/
 .button2:active {
   background-color: #458a8a;
   box-shadow: 2 2px #CDCDCD;
   transform: translateY(2px);
 }


.button3 {
  padding: 10px 20px;
  font-size: 18px;
  text-align: center;
  outline: none;
  color: #fff;
  background-color: #383838;
  border: none;
  border-radius: 10px;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  -khtml-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
  -webkit-tap-highlight-color: rgba(0,0,0,0);
 }
 /*.button3:hover {background-color: #0f8b8d}*/
 .button3:active {
   background-color: #e4e000;
   box-shadow: 2 2px #CDCDCD;
   transform: translateY(2px);
 }


.button4 {
  padding: 5px 30px;
  font-size: 16px;
  text-align: center;
  outline: none;
  color: #fff;
  background-color: #383838;
  border: none;
  border-radius: 15px;
  -webkit-touch-callout: none;
  -webkit-user-select: none;
  -khtml-user-select: none;
  -moz-user-select: none;
  -ms-user-select: none;
  user-select: none;
  -webkit-tap-highlight-color: rgba(0,0,0,0);
 }
 /*.button4:hover {background-color: #0f8b8d}*/
 .button4:active {
   background-color: #e4e000;
   box-shadow: 2 2px #CDCDCD;
   transform: translateY(2px);
 }

 .state {
   font-size: 1.5rem;
   color:#DFFFFF;
   font-weight: bold;
 }

.buttrow>.button2:not(:last-child) {
margin-right: 50px;
margin-bottom: 5px;
}




.gauge {
  position: relative;
  margin: auto;
  background: var(--gauge-bg);
  border: 0.05em solid #150501;
  border-radius: 50%;
  min-width: 250px;
  min-height: 125px;
  font-weight: bold;
  font-size: 1.5rem;
}

.gauge .ticks {
  position: relative;
  width: 100%;
  height: 100%;
  top: 0%;
  left: 0%;
}

.gauge .ticks .min {
  background: black;
  position: relative;
  left: -16%;
  top: -24%;
  width: 100%;
  height: 1%;
  margin-bottom: -1%;
  background: linear-gradient(90deg, rgba(255, 255, 255, 1) 0%, rgba(0, 0, 0, 0) 4%, rgba(0, 0, 0, 1) 4%, rgba(0, 0, 0, 1) 15%, rgba(0, 0, 0, 0) 15%);
  transform: rotate(225deg);
}

.gauge .ticks .mid {
  background: black;
  position: absolute;
  left: 0%;
  top: -50%;
  width: 100%;
  height: 2%;
  margin-bottom: -1%;
  background: linear-gradient(90deg, rgba(255, 255, 255, 1) 0%, rgba(0, 0, 0, 0) 4%, rgba(0, 0, 0, 1) 4%, rgba(0, 0, 0, 1) 15%, rgba(0, 0, 0, 0) 15%);
  transform: rotate(-90deg);
}

.gauge .ticks .max {
  background: black;
  position: relative;
  left: 16%;
  top: -24%;
  width: 100%;
  height: 1%;
  margin-bottom: -1%;
  background: linear-gradient(90deg, rgba(255, 255, 255, 1) 0%, rgba(0, 0, 0, 0) 4%, rgba(0, 0, 0, 1) 4%, rgba(0, 0, 0, 1) 15%, rgba(0, 0, 0, 0) 15%);
  transform: rotate(-45deg);
}



.gauge .tick-circle {
  position: absolute;
  color: #BABABA;
  top: 15%;
  left: 25%;
  width: calc(50% - 0.1em);
  height: calc(70% - 0.1em);
  border-left: 0.1em solid transparent;
  border-top: 0.1em solid transparent;
  border-right: 0.1em solid transparent;
  border-bottom: 0.1em solid;
  border-radius: 50%;
}


:root{
  --gaugevalue: 0;
  --gaugedisplayvalue: 0;
}

.gauge .needle {
  /* Gauge value range 0-100 */
  transform: rotate(calc(45deg * calc(var(--gaugevalue, 0deg) / 45) - 90deg));
  background-color: black;
  position: absolute;
  left: 0%;
  top: 32%;
  width: 100%;
  height: 8%;
  margin-bottom: -4%;
  background: linear-gradient(90deg, rgba(2, 0, 36, 0) 0%, rgba(0, 0, 0, 0) 27%, rgba(15, 139, 141, 1) 27%, rgba(15, 139, 141, 1) 40%, rgba(0, 0, 0, 0) 50%);
}

.gauge .needle .needle-head {
  position: absolute;
  top: 10%;
  left: 26%;
  width: 2.7%;
  height: 80%;
  background-color: #0f8b8d;
  transform: rotate(-45deg);
}

.gauge .labels {
  position: absolute;
  width: 100%;
  height: 100%;
}

.gauge .labels .value-label {
color: #BABABA;
  position: relative;
  top: 10%;
  left: 50%;
  transform: translateX(-50%);
}

.gauge .labels .value-label::after {
  counter-reset: --gaugevalue var(--gaugedisplayvalue);
  content: counter(--gaugevalue);
}

.guide-x, .guide-y {
  background-color: orange;
  visibility: visible;
  position: absolute;
  left: 50%;
  top: 0;
  width: 1px;
  height: 100%;
}

.guide-y {
  left: 0;
  top: 50%;
  width: 100%;
  height: 1px;
}


</style>



</head>

<body>
  <div class="topnav">
    <h1>Plot Device</h1>
  </div>
  <div class="content">
    <div class="card">

<p></p>
      <p><h3>Status</h3></p>
<p><button id="button" class="button"><class="state"><span id="state">$STATE$</span></button></p>



      <h3>Heading</h3>
      <h2><p><span id="heading">$HEAD$</p></span></h2>

      <h3>Course to Steer</h3>
      <h2><p><span id="heading_to_steer">$HTS$</span></p></h2>

 <p>     
<div class="buttrow"> 
<button id="sub10" class="button2">-10</button> 
<button id="add10" class="button2">+10</button>
</div>
</p>
<p>
<div class="buttrow"> 
<button id="sub1" class="button2">-1</button> 
<button id="add1" class="button2">+1</button>
</div>
</p>
<p>
<div class="buttrow"> 
<button id="sub90" class="button2">-90</button>   
<button id="add90" class="button2">+90</button>
</div>
</p>

    </div>
<p><h3><i>Wherever you go, there you are</i></h3></p>
  </div>
<p></p>

<div class="card">
<h2>Pilot Settings</h2>
<h3><p><i>- resets to default on start -</i></p></h3>


<div class="buttrow"> 
<p><span id="R_deadband">Rudder Deadband = $DEADBAND$  <p></p>
<button id="Rdown" class="button4">-</button>   <button id="Rup" class="button4">+</button></span></p>
</div>
<p></p>

<br>
<h3><p>PID Values</p></h3>


<div class="buttrow"> 
<p><span id="K_overall">Gain = $KOVERALL$ <p></p>
<button id="Gdown" class="button4">-</button>   <button id="Gup" class="button4">+</button></span></p>
</div>
<div class="buttrow"> 
<p><span id="K_heading">P = $KHEAD$ <p></p>
<button id="Pdown" class="button4">-</button>   <button id="Pup" class="button4">+</button></span></p>
</div>
<div class="buttrow"> 
<p><span id="K_integral">I = $KINTEGRAL$ <p></p>
<button id="Idown" class="button4">-</button>   <button id="Iup" class="button4">+</button></span></p>
</div>
<div class="buttrow"> 
<p><span id="K_diff">D = $KDIFF$ <p></p>
<button id="Ddown" class="button4">-</button>   <button id="Dup" class="button4">+</button></span></p>
</div>

<br>
<div class="buttrow"> 
<p><span id="MMIN">motorspeed_min = $MMIN$ <p></p>
<button id="MMINdown" class="button4">-</button>   <button id="MMINup" class="button4">+</button></span></p>
</div>


<div class="buttrow"> 
<p><span id="MAGVAR">Magnetic Variation = $MAGVAR$ deg.<p></p>
<button id="magvardown" class="button4">-</button>   <button id="magvarup" class="button4">+</button></span></p>
</div>

<br><br>
<h3><p>Compass Calibration (Min/Max XYZ)</p></h3>

<div class="buttrow"> 
<button id="compcalon" class="button4">$COMPCALIBSTATE$</button>  </span></p>
<p><span id="CALREP"> $CALREP$ <p></p>
</div>




</div> 




<script>






setInterval(function() {
    // Call a function repetatively with X Second interval
    getData();
  }, 1000); //1 Seconds update rate
  
  function getData() {
   websocket.send('readheading');
  }
  
    var gateway = `ws://${window.location.hostname}/ws`;
    var websocket;
    window.addEventListener('load', onLoad);
    function initWebSocket() {
      console.log('Trying to open a WebSocket connection...');
      websocket = new WebSocket(gateway);
      websocket.onopen    = onOpen;
      websocket.onclose   = onClose;
      websocket.onmessage = onMessage; // <-- add this line
    }
    function onOpen(event) {
      console.log('Connection opened');
   
    }
    function onClose(event) {
      console.log('Connection closed');
      setTimeout(initWebSocket, 2000);
    }
  
    function onMessage(event) {
      var obj = JSON.parse(event.data);
      document.getElementById('heading').innerHTML = obj.heading;
      document.getElementById('rudder_position').innerHTML = obj.rudder_position;


     //document.getElementById('gaugevalue').innerHTML = obj.rudder_position;
      //console.log(obj);
      
     // var jgaugevalue = obj.rudder_position;
     // gaugevalue = jgaugevalue;
     // gaugedisplayvalue = jgaugevalue;

      let root = document.documentElement;
      root.style.setProperty('--gaugevalue', obj.rudder_position);
      root.style.setProperty('--gaugedisplayvalue', obj.rudder_position);
      
};
  
  
    function onLoad(event) {
      initWebSocket();
      initButton();
  initsub10();
  initadd10();
  initsub1();
  initadd1();
  initsub90();
  initadd90();
  initRdown();
  initRup();
  initGdown();
  initGup();
  initPdown();
  initPup();
  initIdown();
  initIup();
  initDdown();
  initDup();
  initmagvardown();
  initmagvarup();
  initMMINdown();
  initMMINup();
  initcompcalon();
  initruddbutton();

    }
  
    function initButton() {
      document.getElementById('button').addEventListener('click', toggle);
    }
    function toggle(){
      websocket.send('toggleAP');
    window.location.reload();
    }
  
    function initsub10() {
      document.getElementById('sub10').addEventListener('click', sub10);
    }
    function sub10(){
      websocket.send('subten');
      window.location.reload();
    }
  
    function initadd10() {
      document.getElementById('add10').addEventListener('click', add10);
    }
    function add10(){
      websocket.send('addten');
      window.location.reload();
    }
  
    function initsub1() {
      document.getElementById('sub1').addEventListener('click', sub1);
    }
    function sub1(){
      websocket.send('sub1');
      window.location.reload();
    }
  
    function initadd1() {
      document.getElementById('add1').addEventListener('click', add1);
    }
    function add1(){
      websocket.send('add1');
      window.location.reload();
    }
  
    function initsub90() {
      document.getElementById('sub90').addEventListener('click', sub90);
    }
    function sub90(){
      websocket.send('subninety');
      window.location.reload();
    }
  
    function initadd90() {
      document.getElementById('add90').addEventListener('click', add90);
    }
    function add90(){
      websocket.send('addninety');
      window.location.reload();
    }
  
  
    function initRdown() {
      document.getElementById('Rdown').addEventListener('click', Rdown);
    }
    function Rdown(){
      websocket.send('Rdown');
      window.location.reload();
    }
  
    function initRup() {
      document.getElementById('Rup').addEventListener('click', Rup);
    }
    function Rup(){
      websocket.send('Rup');
      window.location.reload();
    }
  
    function initGdown() {
      document.getElementById('Gdown').addEventListener('click', Gdown);
    }
    function Gdown(){
      websocket.send('Gdown');
      window.location.reload();
    }
  
    function initGup() {
      document.getElementById('Gup').addEventListener('click', Gup);
    }
    function Gup(){
      websocket.send('Gup');
      window.location.reload();
    }
  
    function initPdown() {
      document.getElementById('Pdown').addEventListener('click', Pdown);
    }
    function Pdown(){
      websocket.send('Pdown');
      window.location.reload();
    }
  
    function initPup() {
      document.getElementById('Pup').addEventListener('click', Pup);
    }
    function Pup(){
      websocket.send('Pup');
      window.location.reload();
    }
  
    function initIdown() {
      document.getElementById('Idown').addEventListener('click', Idown);
    }
    function Idown(){
      websocket.send('Idown');
      window.location.reload();
    }
  
    function initIup() {
      document.getElementById('Iup').addEventListener('click', Iup);
    }
    function Iup(){
      websocket.send('Iup');
      window.location.reload();
    }
  
    function initDdown() {
      document.getElementById('Ddown').addEventListener('click', Ddown);
    }
    function Ddown(){
      websocket.send('Ddown');
      window.location.reload();
    }
  
    function initDup() {
      document.getElementById('Dup').addEventListener('click', Dup);
    }
    function Dup(){
      websocket.send('Dup');
      window.location.reload();
    }
  
  function initMMINdown() {
    document.getElementById('MMINdown').addEventListener('click', MMINdown);
  }
  function MMINdown(){
    websocket.send('MMINdown');
    window.location.reload();
  }

function initMMINup() {
    document.getElementById('MMINup').addEventListener('click', MMINup);
  }
  function MMINup(){
    websocket.send('MMINup');
    window.location.reload();
  }

    function initmagvarup() {
      document.getElementById('magvarup').addEventListener('click', magvarup);
    }
    function magvarup(){
      websocket.send('magvarup');
      window.location.reload();
    }
  
    function initmagvardown() {
      document.getElementById('magvardown').addEventListener('click', magvardown);
    }
    function magvardown(){
      websocket.send('magvardown');
      window.location.reload();
    }


    function initcompcalon() {
      document.getElementById('compcalon').addEventListener('click', compcalon);
    }
    function compcalon(){
      websocket.send('compcalon');
      window.location.reload();
    }


        function initruddbutton() {
      document.getElementById('ruddbutton').addEventListener('click', ruddbutton);
    }
    function ruddbutton(){
      websocket.send('toggleRUDD');
      window.location.reload();
    }

</script>

</body>
</html>

)rawliteral";
