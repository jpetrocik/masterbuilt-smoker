import { Router } from 'express';
import { getSmokers } from '../database/database';

const router = Router();

/**
 * @swagger
 * /api:
 *   get:
 *     summary: API Root
 *     description: Returns a welcome message.
 *     responses:
 *       200:
 *         description: Success
 */
router.get('/', (req, res) => {
  res.json({ message: 'API Routes' });
});

/**
 * @swagger
 * /api/smokers:
 *   get:
 *     summary: List registered smokers
 *     description: Returns a list of all registered smokers and their current status.
 *     responses:
 *       200:
 *         description: A list of smokers.
 *         content:
 *           application/json:
 *             schema:
 *               type: array
 *               items:
 *                 type: object
 *                 properties:
 *                   id:
 *                     type: string
 *                     description: The smoker's unique ID.
 *                   status:
 *                     type: string
 *                     description: Current status (online/offline).
 *                   lastSeen:
 *                     type: number
 *                     description: Timestamp of last activity.
 */
router.get('/smokers', (req, res) => {
  const smokers = getSmokers();
  res.json(smokers);
});

export default router;