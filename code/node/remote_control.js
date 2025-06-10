// Remote control of buggy (thanks to Nick Hardy et al.)
// This assumes the ESP32 is a server (receiving stream of data)

const express = require("express");
const app = express();
const dgram = require("dgram");

const PORT1 = 8081;            // Address of node server 
const HOST1 = "192.168.0.246"; // Port of node server

const PORT2 = 3000;            // Address of ESP32 
const HOST2 = "192.168.0.124"; // Port of ESP32 

// Create a new UDP server
const server = dgram.createSocket("udp4");

// Create server that listens on a port
server.on("listening", function () {
  const address = server.address();
  console.log("UDP Server listening on " + address.address + ":" + address.port);
});

server.bind(PORT1, HOST1);

// Create the HTML for the remote control
const controlPanelHTML = `
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Remote Control</title>
  <style>
    body {
      background-color: #FFE4E1;
      font-family: Arial, sans-serif;
    }
    img {
      display: block;
      margin: 50px auto;
      box-shadow: 0px 0px 10px rgba(0,0,0,0.5);
    }
    table {
      border-collapse: collapse;
      margin: 20px auto;
      box-shadow: 0px 0px 10px rgba(0,0,0,0.5);
    }
    th, td {
      border: 1px solid black;
      padding: 10px;
    }
  </style>
</head>
<body>
  <h1>WASD Control</h1>
  <script>
    // Listen for key press events
    document.addEventListener('keydown', function(event) {
      if (event.key === 'w') {
        fetch('/send-message?key=w')
          .then(response => console.log('Forward message sent'))
          .catch(error => console.error(error));
      } else if (event.key === 's') {
        fetch('/send-message?key=s')
          .then(response => console.log('Stop message sent'))
          .catch(error => console.error(error));
      } else if (event.key === 'a') {
        fetch('/send-message?key=a')
          .then(response => console.log('Left message sent'))
          .catch(error => console.error(error));
      } else if (event.key === 'd') {
        fetch('/send-message?key=d')
          .then(response => console.log('Right message sent'))
          .catch(error => console.error(error));
      } else if (event.key === 'q') { // New 'q' key functionality
        fetch('/send-message?key=q')
          .then(response => console.log('Quit message sent'))
          .catch(error => console.error(error));
      }
    });
  </script>
</body>
</html>
`;

// Send the HTML for the control panel
app.get("/", (req, res) => {
  res.send(controlPanelHTML);
});

// Handle the message sending HTTP request
app.get("/send-message", (req, res) => {
  const key = req.query.key;
  if (key === "w") {
    server.send("w", 0, 1, PORT1, HOST2);
    console.log("Forward message sent!");
    res.send("Forward message sent");
  } else if (key === "s") {
    server.send("s", 0, 1, PORT1, HOST2);
    console.log("Stop message sent!");
    res.send("Stop message sent");
  } else if (key === "a") {
    server.send("a", 0, 1, PORT1, HOST2);
    console.log("Left message sent!");
    res.send("Left message sent");
  } else if (key === "d") {
    server.send("d", 0, 1, PORT1, HOST2);
    console.log("Right message sent!");
    res.send("Right message sent");
  } else if (key === "q") { // New 'q' key handling
    server.send("q", 0, 1, PORT1, HOST2);
    console.log("Quit message sent!");
    res.send("Quit message sent");
  } else {
    res.status(400).send("Invalid key");
  }
});

// Listen for requests on ESP32 port
app.listen(PORT2, () => {
  console.log("App listening on port", PORT2);
});
