#ifndef _config_H_
#define _config_H_

//#define SLAVE // Uncomment this only for Slave M5sticks
#define MASTER_HOST "seismoM5"
#define SLAVE_HOST "seismoS1"

//WiFi Parameters
#define WIFI_SSID "XXXXXXX"
#define WIFI_PASS "XXXXXXX"
#ifdef SLAVE
  #define WIFI_HOSTNAME SLAVE_HOST
#else
  #define WIFI_HOSTNAME MASTER_HOST
#endif
#define WIFI_RECONNECT_TIMER 1000// 10 ticks per second

// MQTT Parameters
bool MQTT_active = true; // Enable / Disable MQTT for initial start only

#define MQTT_SERVER "XXXXXXX"
#define MQTT_PORT 1883
#define MQTT_USER "XXXXXXX"
#define MQTT_PASS "XXXXXXX"

#ifdef SLAVE
  #define MQTT_PUB_TOPIC_MAIN MASTER_HOST "/" SLAVE_HOST
  #define MQTT_PUB_STATE_SLAVE (MQTT_PUB_TOPIC_MAIN "/na")
#else
  #define MQTT_PUB_TOPIC_MAIN MASTER_HOST
  #define MQTT_PUB_STATE_SLAVE  MQTT_PUB_TOPIC_MAIN "/" SLAVE_HOST "/state"
#endif

#define MQTT_PUB_EVENT (MQTT_PUB_TOPIC_MAIN "/event")
#define MQTT_PUB_STATE  (MQTT_PUB_TOPIC_MAIN "/state")
#define MQTT_PUB_AVAILABILITY (MQTT_PUB_TOPIC_MAIN "/status") // online or offline
#define MQTT_PUB_PGA_TRIGGER (MQTT_PUB_TOPIC_MAIN "/pga_trigger") // change pga_trigger
#define MQTT_PUB_COMMAND (MQTT_PUB_TOPIC_MAIN "/command")

// SPK HAT, default disabled
#define SPK_pin 26
#define spkChannel 0
#define spkFreq 50
#define spkResolution 10

// MPU6886 Calibration Parameters
#define buffersize 1000     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
#define acel_deadzone 7     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
#define scale_factor 1      // scale_factor
#define eq_pet 40           // Post Event Time for PGA Trigger (4 seconds)
#define gravity 16384.0     // Gravity
//STA/LTA
#define trigger_threshold 4
#define detrigger_threshold 2

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
  <head>
    <title>SeismoM5 Web Server Configuration</title>
    <meta charset='utf-8'>
    <meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no" />
    <link rel="icon" href="data:,">
    <style>
      html {
        font-family: verdana;
        text-align: center;
      }

      h1 {
        font-size: 1.8rem;
        color: white;
      }

      h2 {
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
      <h1 id = 'header'> SeismoM5 Web Server </h3>
        <h2> Configuration </h2>
    </div>
    <div style='text-align:left;display:inline-block;min-width:340px;'>
      <fieldset>
        <form method='get' action='sv' autocomplete="off">
          <fieldset>
            <legend>
              <b>&nbsp;MQTT parameters&nbsp;</b>
            </legend>
            <label for="new_pga">PGA Trigger (g)</label><br>
            <input id="new_pga" name="new_pga" maxlength="6" value=%PGAPLACEHOLDER%>
            <br />
            <br />
            <label for="new_bri">LCD Standby Brightness (7-15)</label><br>
            <input id="new_bri" name="new_bri" maxlength="2" value=%BRIPLACEHOLDER%>
            <br />
            <br />
            <label for="new_per">Update Period (sec)</label><br>
            <input id="new_per" name="new_per" maxlength="4" value=%PERPLACEHOLDER%>
            <br />
            <br />
            <input type="checkbox" id='slta' name='slta' onclick="disablePga();"> <label for="slta">Use STA/LTA Method</label><br>
            <br />
          </fieldset>
          <fieldset>
            <legend>
              <b>&nbsp;Other parameters&nbsp;</b>
            </legend>
            <input type="checkbox" id='spk' name='spk'> <label for="spk">Speaker Enable</label><br>
            <br />
            <input type="checkbox" id='con' name='con'> <label for="con">Continuous Graph</label><br>
            <br />
            <label for="lg">Logging:</label> <select id='lg' name='lg'>
              <option value='0'>None</option>
              <option value='1'>Serial Only</option>
              <option value='2'>WebSerial Only</option>
              <option value='3'>Both</option>
            </select>
          </fieldset>
          <fieldset>
            <legend>
              <b>&nbsp;Master / Slave Config&nbsp;</b>
            </legend>
            <input type="checkbox" id='master' name='master'> <label for="master">Master</label>
          </fieldset>
          <div class="button-container-div">
            <button type='submit' class='button bgrn'>Save</button>
          </div>
        </form>
      </fieldset>
      <br />
      <div class="button-container-div">
        <button onclick="window.location.href='/webserial';"> Web Serial</button>
      </div>
      <br />
      <br />
      <form action='recal' method='get'>
        <div class="button-container-div">
          <button class='button bred'>Recalibrate MPU</button>
        </div>
      </form>
      <br />
      <br />
      <form action='rt' method='get' onsubmit='return confirm("Confirm Reset M5");'>
        <div class="button-container-div">
          <button class='button bred'>Reset M5</button>
        </div>
      </form>
      <br />
      <br />
    </div>
    <script>
      document.getElementById('lg').value = %LOGPLACEHOLDER%;
	    document.getElementById('spk').checked = %SPKPLACEHOLDER%;
	    document.getElementById('con').checked = %CONPLACEHOLDER%;
	    document.getElementById('slta').checked = %SLTAPLACEHOLDER%;
	    document.getElementById('master').checked = %MASTERPLACEHOLDER%;
      document.getElementById('new_pga').readOnly = document.getElementById('slta').checked;
      if (%MASTERPLACEHOLDER%)
      {
        document.getElementById("header").innerHTML = "SeismoM5 - Master";
      }
      if (%SLAVEPLACEHOLDER%)
      {
        document.getElementById('master').disabled = true;
        document.getElementById("header").innerHTML = "SeismoM5 - Slave";
      }
      function disablePga()
      {
        if (document.getElementById('slta').checked) 
        {
            document.getElementById('new_pga').readOnly = true;
        } else {
            document.getElementById('new_pga').readOnly = false;
        }
      }
    </script>
  </body>
</html>
)rawliteral";

#endif
