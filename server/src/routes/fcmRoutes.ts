// server/src/routes/fcmRoutes.ts
import { Router } from 'express';
import { handleRegisterFcmToken } from '../controllers/fcmController';

const router = Router();

router.post('/register', handleRegisterFcmToken);

export default router;
