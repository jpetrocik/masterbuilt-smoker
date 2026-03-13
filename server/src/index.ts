import express from 'express';
import cors from 'cors';
import dotenv from 'dotenv';
import http from 'http';
import swaggerUi from 'swagger-ui-express';
import swaggerJsdoc from 'swagger-jsdoc';
import routes from './routes';
import fcmRoutes from './routes/fcmRoutes';
import config from './config';
import { initDatabase } from './database/database';
import { connectMqtt } from './mqtt/mqttService';
import { createWebSocketServer } from './websocket/websocketServer';
import { initNotificationService } from './notifications/notificationService';

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

// Root route
app.get('/', (req, res) => {
  res.json({ message: 'Welcome to the API' });
});

// Connect to MQTT broker
connectMqtt();

// Initialize Notification Service
initNotificationService();

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
  
  app.use('/api-docs', swaggerUi.serve, swaggerUi.setup(swaggerSpec));
  console.log(`Swagger UI available at http://localhost:${PORT}/api-docs`);
}

// Create HTTP server
const server = http.createServer(app);

// Create WebSocket server
createWebSocketServer(server);

// Start server
server.listen(PORT, () => {
  console.log(`Server is running on http://localhost:${PORT}`);
});