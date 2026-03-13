// server/src/controllers/fcmController.ts
import { Request, Response } from 'express';
import { registerFcmToken } from '../database/database';

export function handleRegisterFcmToken(req: Request, res: Response): void {
  const { smokerId, token } = req.body;

  if (!smokerId || !token) {
    res.status(400).send({ error: 'smokerId and token are required' });
    return;
  }

  try {
    registerFcmToken(smokerId, token);
    res.status(200).send({ success: true });
  } catch (error) {
    console.error('Failed to register FCM token:', error);
    res.status(500).send({ error: 'Failed to register token' });
  }
}
