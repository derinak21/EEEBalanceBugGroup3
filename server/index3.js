const express = require('express');
const app = express();
const PORT = 3001;
const bodyParser = require('body-parser');
const http = require('http');
const WebSocket = require('ws');
const cors = require('cors');

app.use(cors());
app.use(cors({
    methods: ['GET','POST','DELETE','UPDATE','PUT','PATCH']
   }));
app.use(bodyParser.json());
app.use(cors({
 origin: '*'
}));

var mysql = require('mysql');
var con = mysql.createConnection(
{
 host: "54.172.241.111",
 user: "username",
 password: "userpwd",
 database: "MapsDB"
});
con.connect(function(err) {
 if (err) throw err;
 console.log("Successfully connected to the database...\n");
});


const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

wss.on('connection', (ws) => {
    console.log(`⚡: New user connected!`);

    ws.on('message', (message) => {
        const coordinatesData = message.toString();
        const coordinates = JSON.parse(coordinatesData);
        // Access the individual coordinates
        const x = coordinates.x;
        const y = coordinates.y;

        const data = { x, y };
        const jsonData = JSON.stringify(data);
        console.log('Received coordinates:', x, y);
        wss.clients.forEach((client) => {
            if (client.readyState === WebSocket.OPEN) {
              client.send(jsonData);
            }
        });
    });

    ws.on('close', () => {
        console.log('🔥: A user disconnected');
    });
});

app.post('/', (req, res) => {
    const { username, password } = req.body;
    
    if (username === 'group3' && password === 'alina') {
        // Authentication successful, send token
        res.json({ token: 'test123' });
        console.log('correct')
    } else {
        // Authentication failed
        res.status(401).json({ error: 'Invalid credentials' });
    }
    });

app.get('/command', (req, res) =>{
    console.log('received from rover');
    const response = {
        'value1': 10.2,
        'value2': 20.9,
        'value3': 18.7
      };
    
      res.json(response);
});

app.get("/mapsQuery", (req, res) => {
    con.query("SELECT DISTINCT map_id, map_name, creation_date FROM MapArchive", function (err, result, fields) {
    if (err) throw err;
    res.json(result)
    });
});

app.post('/replay', (req, res) => {
    const { mapId } = req.body;
  
    con.query(`SELECT Rx, Ry FROM MapArchive WHERE map_id = ${mapId}`, function (err, result, fields) {
      if (err) {
        console.error(err);
        res.status(500).json({ error: 'An error occurred' });
      } else {
        const coordinates = result.map((row) => ({
            x: row.Rx,
            y: row.Ry,
          }));
    
          res.json(coordinates);
          console.log(coordinates);
      }
    });
  });

app.post('/submitMap', (req, res) => {
    const { mapId } = req.body;

    con.query(`SELECT Rx, Ry FROM MapArchive WHERE map_id = ${mapId}`, function (err, result, fields) {
        if (err) {
        console.error(err);
        res.status(500).json({ error: 'An error occurred' });
        } else {
        const coordinates = result.map((row) => ({
            x: row.Rx,
            y: row.Ry,
            }));

            res.json(coordinates);
            console.log(coordinates);
        }
    });
});

server.listen(PORT, () => {
  console.log(`Server listening on ${PORT}`);
});