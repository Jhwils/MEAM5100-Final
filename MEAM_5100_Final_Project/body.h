const char body[] PROGMEM = R"===(
  <!DOCTYPE html>
  <html>
  <head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
  .slidecontainer {
    width: 100%;
  }

  .slider {
    -webkit-appearance: none;
    width: 100%;
    height: 25px;
    background: #d3d3d3;
    outline: none;
    opacity: 0.7;
    -webkit-transition: .2s;
    transition: opacity .2s;
  }

  .slider:hover {
    opacity: 1;
  }

  .slider::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 25px;
    height: 25px;
    background: #04AA6D;
    cursor: pointer;
  }

  .slider::-moz-range-thumb {
    width: 25px;
    height: 25px;
    background: #04AA6D;
    cursor: pointer;
  }
  </style>
  </head>
  <body>

  <h1>Website for 4.1.3b</h1>

  <div class="slidecontainer">
    <p>Drag the slider to change the frequency. Current frequency: <span id="freq"></span></p>
    <input type="range" min="4" max="25" value="15" class="slider" id="myFrequency">
  </div>


  <div class="slidecontainer">
    <p>Drag the slider to change the duty cycle. Current duty cycle raw value: <span id="duty"></span></p>
    <input type="range" min="0" max="16383" value="8192" class="slider" id="myDutyCycle">
  </div>

  <script>
  var frequencySlider = document.getElementById("myFrequency");
  var dutyCycleSlider = document.getElementById("myDutyCycle");
  var frequencyOutput = document.getElementById("freq");
  var dutyCycleOutput = document.getElementById("duty");
  frequencyOutput.innerHTML = frequencySlider.value;
  dutyCycleOutput.innerHTML = dutyCycleSlider.value;

  frequencySlider.oninput = function() {
    frequencyOutput.innerHTML = this.value;
  }
  dutyCycleSlider.oninput = function() {
    dutyCycleOutput.innerHTML = this.value;
  }
  </script>

  </body>
  </html> 
)===";