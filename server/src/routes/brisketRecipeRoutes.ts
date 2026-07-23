// server/src/routes/brisketRecipeRoutes.ts
import { Router } from 'express';
import { handleStartBrisketRecipe, handleStopBrisketRecipe, handleGetBrisketRecipeStatus } from '../controllers/brisketRecipeController';

const router = Router();

router.post('/:smokerId/start', handleStartBrisketRecipe);
router.post('/:smokerId/stop', handleStopBrisketRecipe);
router.get('/:smokerId/status', handleGetBrisketRecipeStatus);

export default router;
