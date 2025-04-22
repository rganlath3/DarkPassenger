// webpage.h - Contains the HTML for the web interface
#ifndef WEBPAGE_H
#define WEBPAGE_H

// HTML for the web page (stored in PROGMEM to save RAM)
const char webpageTemplate[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <title>Dark Passenger</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    :root {
      --primary-color: #4CAF50;
      --danger-color: #f44336;
      --info-color: #2196F3;
      --bg-color: #f2f2f2;
      --text-color: #333;
      --button-radius: 8px;
      --container-radius: 10px;
      --spacing-sm: 8px;
      --spacing-md: 15px;
      --spacing-lg: 20px;
    }
    
    * {
      box-sizing: border-box;
      margin: 0;
      padding: 0;
    }
    
    html {
      font-family: Arial, sans-serif;
      font-size: 16px;
    }
    
    body {
      margin: 0 auto;
      padding: var(--spacing-lg);
      max-width: 600px;
      text-align: center;
    }
    
    h1 {
      font-size: 1.5rem;
      margin-bottom: var(--spacing-lg);
    }
    
    h2 {
      font-size: 1.2rem;
      margin-bottom: var(--spacing-md);
    }
    
    .control-grid {
      display: grid;
      grid-template-columns: repeat(3, 1fr);
      grid-template-rows: repeat(3, 1fr);
      gap: var(--spacing-sm);
      margin: var(--spacing-lg) auto;
      max-width: 300px;
    }
    
    .grid-container {
      display: grid;
      grid-template-columns: 1fr;
      gap: var(--spacing-sm);
      margin: var(--spacing-lg) 0;
    }
    
    @media (min-width: 480px) {
      .grid-container {
        grid-template-columns: repeat(3, 1fr);
      }
      
      h1 {
        font-size: 1.8rem;
      }
    }
    
    .button {
      background-color: var(--primary-color);
      border: none;
      color: white;
      padding: var(--spacing-md);
      text-decoration: none;
      font-size: 0.9rem;
      cursor: pointer;
      border-radius: var(--button-radius);
      transition: background-color 0.3s, transform 0.1s;
      touch-action: manipulation;
      -webkit-tap-highlight-color: transparent;
    }
    
    .button:active {
      transform: scale(0.95);
    }
    
    .stop {
      background-color: var(--danger-color);
      font-weight: bold;
    }
    
    .data-container {
      background-color: var(--bg-color);
      border-radius: var(--container-radius);
      padding: var(--spacing-md);
      margin: var(--spacing-lg) 0;
      text-align: left;
    }
    
    .center-cell {
      grid-column: 2;
      grid-row: 2;
    }
    
    .empty-cell {
      visibility: hidden;
    }
    
    #refreshIcon {
      position: fixed;
      bottom: var(--spacing-lg);
      right: var(--spacing-lg);
      font-size: 24px;
      cursor: pointer;
      background-color: var(--info-color);
      color: white;
      width: 50px;
      height: 50px;
      border-radius: 50%;
      display: flex;
      justify-content: center;
      align-items: center;
      box-shadow: 0 2px 5px rgba(0,0,0,0.2);
      z-index: 100;
    }
    
    #batteryStatus, #encoderData, #gpsData, #imsData {
      margin-bottom: var(--spacing-sm);
      word-break: break-word;
    }
  </style>
</head>
<body>
  <h1>Dark Passenger Web Controller</h1>
  
  <div class="control-grid">
    <div class="empty-cell"></div>
    <button class="button" onclick="sendCommand('F')">Forward</button>
    <div class="empty-cell"></div>
    <button class="button" onclick="sendCommand('L')">Left</button>
    <button class="button stop" onclick="sendCommand('S')">STOP</button>
    <button class="button" onclick="sendCommand('R')">Right</button>
    <div class="empty-cell"></div>
    <button class="button" onclick="sendCommand('B')">Backward</button>
    <div class="empty-cell"></div>
  </div>
  
  <div class="grid-container">
    <button class="button" onclick="sendCommand('H')">Toggle Headlight</button>
    <button class="button" onclick="sendCommand('J')">Toggle Brake Light</button>
    <button class="button" onclick="sendCommand('C')">Clear Counters</button>
  </div>
  
  <div class="grid-container">
    <button class="button" onclick="sendCommand('G')">Get GPS</button>
    <button class="button" onclick="sendCommand('I')">Get IMS</button>
    <button class="button" onclick="sendCommand('E')">Get Distance</button>
  </div>
  
  <div class="data-container">
    <h2>Status</h2>
    <p id="batteryStatus">%BATTERY%</p>
    <div id="encoderData">%ENCODER%</div>
    <div id="gpsData">%GPS%</div>
    <div id="imsData">%IMS%</div>
  </div>
  
  <div id="refreshIcon" onclick="location.reload()">&#x21bb;</div>
  
  <script>
    function sendCommand(cmd) {
      fetch('/' + cmd)
        .then(response => {
          if (cmd !== 'F' && cmd !== 'B' && cmd !== 'L' && cmd !== 'R' && cmd !== 'S') {
            setTimeout(() => location.reload(), 500);
          }
        })
        .catch(error => console.error('Error sending command:', error));
    }
    
    // Add touch feedback for mobile
    const buttons = document.querySelectorAll('.button');
    buttons.forEach(button => {
      button.addEventListener('touchstart', function() {
        this.style.opacity = '0.7';
      });
      button.addEventListener('touchend', function() {
        this.style.opacity = '1';
      });
    });
  </script>
</body>
</html>
)rawliteral";

#endif