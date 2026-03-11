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
 *     description: Returns a list of registered smokers filtered by status.
 *     parameters:
 *       - in: query
 *         name: status
 *         schema:
 *           type: string
 *           enum: [all, online, offline]
 *         description: Filter by status. Defaults to 'online' if not provided.
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
  const statusParam = req.query.status as string;
  let filterStatus: 'online' | 'offline' | 'all';
  
  if (!statusParam || statusParam === 'online') {
    filterStatus = 'online';
  } else if (statusParam === 'offline') {
    filterStatus = 'offline';
  } else if (statusParam === 'all') {
    filterStatus = 'all';
  } else {
    return res.status(400).json({ error: 'Invalid status parameter. Use all, online, or offline.' });
  }
  
  const smokers = getSmokers(filterStatus);
  res.json(smokers);
});

export default router;