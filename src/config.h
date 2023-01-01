#ifdef STICKC
  #include <M5StickC.h>
#else
  #include <M5StickCPlus.h>
#endif

// Logging
uint8_t logging = 3; // 0: None, 1: Serial only, 2: WebSerial only, 3: Serial and WebSerial

//WiFi Parameters
#define WIFI_SSID "XXXXXXXX"
#define WIFI_PASS "XXXXXXXX"

// MQTT Parameters
bool MQTT_active = true; // Enable / Disable MQTT for initial start only

#define MQTT_SERVER "XXXXXXXX"
#define MQTT_PORT "1883"
#define MQTT_USER "XXXXXXXX"
#define MQTT_PASS "XXXXXXXX"

#define MQTT_PUB_TOPIC_MAIN "m5seismo"

#define MQTT_PUB_EVENT (MQTT_PUB_TOPIC_MAIN "/event")
#define MQTT_PUB_STATE  (MQTT_PUB_TOPIC_MAIN "/state")
#define MQTT_PUB_AVAILABILITY (MQTT_PUB_TOPIC_MAIN "/status") // online or offline
#define MQTT_PUB_PGA_TRIGGER (MQTT_PUB_TOPIC_MAIN "/pga_trigger") // change pga_trigger
#define MQTT_PUB_COMMAND (MQTT_PUB_TOPIC_MAIN "/command")

// SPK HAT, default disabled
bool SPK_HAT;
#define SPK_pin 26
#define spkChannel 0
#define spkFreq 50
#define spkResolution 10

// Screen coordinates
int8_t lcd_brightness; // TFT backlight brightness for standby ( value: 7 - 15 )
uint8_t graph_x_axis = 7; // X Coordinate for Vertical axis line
uint8_t graph_x_start = 8; // X Coordinate for where the graph starts (increases)
bool continuous_graph; // false: Draw graph only when EQ happens
uint8_t previous_graph_y[3];
#ifdef STICKC
  // Graph coordinates for main screen: M5StickC: 160x80
  uint8_t graph_y_axis[3] = {25,35,45}; // Y coordinates for X,Y,Z horizontal axis lines
  uint8_t graph_y_axis_boundary = 5; // pixels
  uint8_t graph_x_limit = 155; // X Coordinate limit for graph
  uint8_t graph_scale = 50;
  uint8_t graph_clear_y_height = 37;
  uint8_t pga_print_x = 120;
  uint8_t pga_print_y = 0;
  uint8_t mqtt_print_x = 130; // 120 for text size 1
  uint8_t mqtt_print_y = 70;
#else
  // Graph coordinates for main screen: M5StickCPlus: 240x135 pixels
  uint8_t graph_y_axis[3] = {35,65,95}; // Y coordinates for X,Y,Z horizontal axis lines
  uint8_t graph_y_axis_boundary = 15; // pixels
  uint8_t graph_x_limit = 235; // X Coordinate limit for graph
  uint8_t graph_scale = 150;
  uint8_t graph_clear_y_height = 97;
  uint8_t pga_print_x = 200;
  uint8_t pga_print_y = 2;
  uint8_t mqtt_print_x = 200;
  uint8_t mqtt_print_y = 125;
#endif

// MPU6886 Calibration Parameters
#define buffersize 1250     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
#define acel_deadzone 8     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
#define scale_factor 1      // scale_factor
#define eq_pet 40           // Post Event Time (Around 5 secs)

const char update_html[] PROGMEM = R"rawliteral(
<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>
<title>SeismoM5</title>
<h1>SeismoM5 OTA</h1>
<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>
<input type='file' name='update' id='file' onchange='sub(this)' style=display:none>
<label id='file-input' for='file'>   Choose file...</label>
<input type='submit' class=btn value='Update'>
<br><br>
<div id='prg'></div>
<br><div id='prgbar'><div id='bar'></div></div><br></form>
<script>
function sub(obj){
var fileName = obj.value.split('\\\\');
document.getElementById('file-input').innerHTML = '   '+ fileName[fileName.length-1];
};
$('form').submit(function(e){
e.preventDefault();
var form = $('#upload_form')[0];
var data = new FormData(form);
$.ajax({
url: '/doUpdate',
type: 'POST',
data: data,
contentType: false,
processData:false,
xhr: function() {
var xhr = new window.XMLHttpRequest();
xhr.upload.addEventListener('progress', function(evt) {
if (evt.lengthComputable) {
var per = evt.loaded / evt.total;
$('#prg').html('progress: ' + Math.round(per*100) + '%');
$('#bar').css('width',Math.round(per*100) + '%');
}
}, false);
return xhr;
},
success:function(d, s) {
console.log('success!') 
},
error: function (a, b, c) {
}
});
});
</script>
<style>#file-input,input{width:100%;height:44px;border-radius:4px;margin:10px auto;font-size:15px}
input{background:#f1f1f1;border:0;padding:0 15px}body{background:#3498db;font-family:sans-serif;font-size:14px;color:#777}
#file-input{padding:0;border:1px solid #ddd;line-height:44px;text-align:left;display:block;cursor:pointer}
#bar,#prgbar{background-color:#f1f1f1;border-radius:10px}#bar{background-color:#3498db;width:0%;height:10px}
form{background:#fff;max-width:258px;margin:75px auto;padding:30px;border-radius:5px;text-align:center}
.btn{background:#3498db;color:#fff;cursor:pointer}h1{color:white;text-align:center;}</style>
)rawliteral";

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
	<title>SeismoM5 Web Server Configuration</title>
	<meta charset='utf-8'>
	<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no" />
	<link rel="icon" href="data:,">
	<style>		html {
			  font-family: verdana;
		  text-align: center;
		}
		h1 {
		  font-size: 1.8rem;
		  color: white;
		}
		h2{
		  font-size: 1.2rem;
		  color: white;
		}
		.topnav {
		  overflow: hidden;
		  background-color: #1fa3ec;
		  text-align: center;
		}
		div,
		fieldset,
		input,
		select {
			  padding: 5px;
			  font-size: 1em;
		}
		
		input {
			  box-sizing: border-box;
			  -webkit-box-sizing: border-box;
			  -moz-box-sizing: border-box;
		}
				
		textarea {
			  resize: none;
			  height: 318px;
			  padding: 5px;
			  overflow: auto;
		}
		
		body {
			  text-align: center;
			  font-family: verdana;
		}
		
		td {
			  padding: 0px;
		}
		
		button {
			  border: 0;
			  border-radius: 0.3rem;
			background-color: #1fa3ec;
			color: #fff;
			line-height: 2.4rem;
			font-size: 1.2rem;
			-webkit-transition-duration: 0.4s;
			transition-duration: 0.4s;
			cursor: pointer;
		}
		button:hover {
			background-color: #0e70a4;
		}
    .button-container-div {
            text-align: center;
    }
		.bred {
			background-color: #d43535;
		}
		
		.bred:hover {
			background-color: #931f1f;
		}
		
		.bgrn {
			  background-color: #47c266;
		}
		
		.bgrn:hover {
			  background-color: #5aaf6f;
		}
		
		a {
			  text-decoration: none;
		}
		
		.p {
			  float: left;
			  text-align: left;
		}
		
		.q {
			  float: right;
			  text-align: right;
		}
    .form-control {
        display: inline-block;
    }

    .btn {
       display: inline-block;
    }
	</style>
</head>
<body onload="clearInp()">
<div class="topnav">
	<h1>
		SeismoM5 Web Server
		</h3>
		<h2>
			Configuration
		</h2>
</div>
	<br/>
    <div class="button-container-div">
      <button onclick="window.location.href='/webserial';">
        Web Serial</button>
    </div>
	<br/>
    <div class="button-container-div">
      <button onclick="window.location.href='/update';">
        OTA Firmware Update</button>
    </div>
	<br/>
  <form action="/cp">
    <label for="fname">PGA Trigger:</label>
    <input type="text" id="new_pga" name="new_pga" maxlength="6" size="2" placeholder=%PGAPLACEHOLDER%> <button>Change PGA</button>
	</form>
	<br/>
	<br/>
	<form action='rt' method='get' onsubmit='return confirm("Confirm Reset M5");'>
    <div class="button-container-div">
		  <button class='button bred'>Reset M5  for Recalibration</button>
    </div>
	</form>
	<br/>
	<br/>
</div>
<script>
  function clearInp() {
    document.getElementById("new_pga").value = "";
  }
</script>
</body>
</html>
)rawliteral";
