import express from 'express';
import cors from 'cors';
import dotenv from 'dotenv';
import http from 'http';
import swaggerUi from 'swagger-ui-express';
import swaggerJsdoc from 'swagger-jsdoc';
import path from 'path';
import routes from './routes';
import fcmRoutes from './routes/fcmRoutes';
import config from './config';
import { initDatabase } from './database/database';
import { connectMqtt } from './mqtt/mqttService';
import { createWebSocketServer } from './websocket/websocketServer';
import { initNotificationService } from './notifications/notificationService';
import { initCollagenService } from './collagen/collagenService';

// Load environment variables
dotenv.config();

// Initialize database
initDatabase();

const app = express();
const PORT = process.env.PORT || config.server.port;

// Middleware
app.use(cors());
app.use(express.json());
app.use(express.urlencoded({ extended: true }));

// Health check route
app.get('/health', (req, res) => {
  res.status(200).json({ status: 'OK', timestamp: new Date().toISOString() });
});

// API Routes
app.use('/api', routes);
app.use('/api/fcm', fcmRoutes);

// Connect to MQTT broker
connectMqtt();

// Initialize Notification Service
initNotificationService();

// Initialize Collagen Service
initCollagenService();

// Swagger setup (development only)
if (process.env.NODE_ENV === 'development') {
  const swaggerOptions = {
    definition: {
      openapi: '3.0.0',
      info: {
        title: 'Multitenant Smoker Telemetry API',
        version: '1.0.0',
        description: 'API for managing smoker telemetry data',
      },
      servers: [
        {
          url: `http://localhost:${PORT}`,
          description: 'Development server',
        },
      ],
    },
    apis: ['./src/routes/*.ts'], // Path to the API docs
  };

  const swaggerSpec = swaggerJsdoc(swaggerOptions);
  
  app.get('/swagger.json', (req, res) => {
    res.json(swaggerSpec);
  });

  app.use('/api-docs', swaggerUi.serve, swaggerUi.setup(swaggerSpec, { swaggerUrl: '/swagger.json' }));
  console.log(`Swagger UI available at http://localhost:${PORT}/api-docs`);
}

// Serve static files from the React app
app.use(express.static(path.join(__dirname, '../../www/dist')));

// All other requests return the React app, so it can handle routing
app.use((req, res) => {
  res.sendFile(path.join(__dirname, '../../www/dist', 'index.html'));
});

// Create HTTP server
const server = http.createServer(app);

// Create WebSocket server
createWebSocketServer(server);

// Start server
server.listen(PORT, () => {
  console.log(`Server is running on http://localhost:${PORT}`);
});