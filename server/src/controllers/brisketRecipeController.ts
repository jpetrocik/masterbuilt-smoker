// server/src/controllers/brisketRecipeController.ts
import { Request, Response } from 'express';
import { startBrisketRecipe, stopBrisketRecipe, getBrisketRecipeState } from '../brisket/brisketRecipeService';

export function handleStartBrisketRecipe(req: Request, res: Response): void {
  const smokerId = Array.isArray(req.params.smokerId) ? req.params.smokerId[0] : req.params.smokerId;

  if (!smokerId) {
    res.status(400).json({ error: 'smokerId is required' });
    return;
  }

  try {
    startBrisketRecipe(smokerId);
    res.status(200).json({ success: true, message: `Brisket recipe started for smoker ${smokerId}` });
  } catch (error) {
    console.error('Failed to start brisket recipe:', error);
    res.status(500).json({ error: 'Failed to start brisket recipe' });
  }
}

export function handleStopBrisketRecipe(req: Request, res: Response): void {
  const smokerId = Array.isArray(req.params.smokerId) ? req.params.smokerId[0] : req.params.smokerId;

  if (!smokerId) {
    res.status(400).json({ error: 'smokerId is required' });
    return;
  }

  try {
    stopBrisketRecipe(smokerId);
    res.status(200).json({ success: true, message: `Brisket recipe stopped for smoker ${smokerId}` });
  } catch (error) {
    console.error('Failed to stop brisket recipe:', error);
    res.status(500).json({ error: 'Failed to stop brisket recipe' });
  }
}

export function handleGetBrisketRecipeStatus(req: Request, res: Response): void {
  const smokerId = Array.isArray(req.params.smokerId) ? req.params.smokerId[0] : req.params.smokerId;

  if (!smokerId) {
    res.status(400).json({ error: 'smokerId is required' });
    return;
  }

  try {
    const state = getBrisketRecipeState(smokerId);
    if (!state) {
      res.status(200).json({ active: false, message: `No active brisket recipe for smoker ${smokerId}` });
    } else {
      res.status(200).json(state);
    }
  } catch (error) {
    console.error('Failed to get brisket recipe status:', error);
    res.status(500).json({ error: 'Failed to get brisket recipe status' });
  }
}
