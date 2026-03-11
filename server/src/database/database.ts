import { DatabaseSync } from 'node:sqlite';
import path from 'path';
import config from '../config';
import { SmokerTelemetryPayload, CookHistoryRow } from '../types';

// Initialize database
const dbPath = path.resolve(config.database.path);
const db = new DatabaseSync(dbPath);

export function initDatabase(): void {
  console.log(`Initializing database at ${dbPath}`);
  
  // Enable WAL mode for better concurrency
  db.exec('PRAGMA journal_mode = WAL;');
  
  // Create smokers table
  db.exec(`
    CREATE TABLE IF NOT EXISTS smokers (
      id TEXT PRIMARY KEY,
      status TEXT NOT NULL,
      last_seen INTEGER NOT NULL
    )
  `);
  
  // Create cook_history table
  db.exec(`
    CREATE TABLE IF NOT EXISTS cook_history (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      smoker_id TEXT NOT NULL,
      timestamp INTEGER NOT NULL,
      temperature REAL,
      target_temperature REAL,
      cook_timer INTEGER,
      probe1 REAL,
      target_probe1 REAL,
      probe2 REAL,
      target_probe2 REAL,
      probe3 REAL,
      target_probe3 REAL,
      probe4 REAL,
      target_probe4 REAL,
      FOREIGN KEY (smoker_id) REFERENCES smokers(id) ON DELETE CASCADE
    )
  `);
  
  // Create index for faster queries
  db.exec(`
    CREATE INDEX IF NOT EXISTS idx_cook_history_smoker_timestamp 
    ON cook_history(smoker_id, timestamp)
  `);
}

export function insertTelemetry(smokerId: string, payload: SmokerTelemetryPayload): void {
  const timestamp = Date.now();
  
  // Insert into cook_history
  const stmt = db.prepare(`
    INSERT INTO cook_history (
      smoker_id, timestamp, temperature, target_temperature, cook_timer,
      probe1, target_probe1, probe2, target_probe2,
      probe3, target_probe3, probe4, target_probe4
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
  `);
  
  stmt.run(
    smokerId,
    timestamp,
    payload.temperature,
    payload.targetTemperature,
    payload.cookTimer,
    payload.probe1 ?? null,
    payload.targetProbe1 ?? null,
    payload.probe2 ?? null,
    payload.targetProbe2 ?? null,
    payload.probe3 ?? null,
    payload.targetProbe3 ?? null,
    payload.probe4 ?? null,
    payload.targetProbe4 ?? null
  );
  
  // Update smoker status
  updateSmokerStatus(smokerId, 'online');
}

export function updateSmokerStatus(smokerId: string, status: 'online' | 'offline'): void {
  const timestamp = Date.now();
  
  const stmt = db.prepare(`
    INSERT OR REPLACE INTO smokers (id, status, last_seen)
    VALUES (?, ?, ?)
  `);
  
  stmt.run(smokerId, status, timestamp);
}

export function getSmokers(filterStatus?: 'online' | 'offline' | 'all'): { id: string; status: string; lastSeen: number }[] {
  let sql = 'SELECT id, status, last_seen as lastSeen FROM smokers';
  
  if (filterStatus && filterStatus !== 'all') {
    sql += ' WHERE status = ?';
    const stmt = db.prepare(sql);
    return stmt.all(filterStatus) as unknown as { id: string; status: string; lastSeen: number }[];
  }
  
  const stmt = db.prepare(sql);
  return stmt.all() as unknown as { id: string; status: string; lastSeen: number }[];
}

export function getHistoricalData(smokerId: string, since: number): CookHistoryRow[] {
  const stmt = db.prepare(`
    SELECT * FROM cook_history 
    WHERE smoker_id = ? AND timestamp >= ? 
    ORDER BY timestamp ASC
  `);
  
  // Cast the result to CookHistoryRow[] since we know the schema matches
  return stmt.all(smokerId, since) as unknown as CookHistoryRow[];
}

export function purgeOldHistory(retentionHours: number, offlinePurgeHours: number): void {
  const now = Date.now();
  const globalCutoff = now - (retentionHours * 60 * 60 * 1000);
  const offlineCutoff = now - (offlinePurgeHours * 60 * 60 * 1000);
  
  // Purge old history based on timestamp (global retention)
  const globalStmt = db.prepare('DELETE FROM cook_history WHERE timestamp < ?');
  globalStmt.run(globalCutoff);
  
  // Purge history for smokers that have been offline for too long
  // First, get smokers that are offline and last_seen < offlineCutoff
  const offlineSmokersStmt = db.prepare(`
    SELECT id FROM smokers 
    WHERE status = 'offline' AND last_seen < ?
  `);
  const offlineSmokers = offlineSmokersStmt.all(offlineCutoff) as { id: string }[];
  
  if (offlineSmokers.length > 0) {
    const smokerIds = offlineSmokers.map(s => s.id);
    
    // Delete history for these smokers
    const placeholders = smokerIds.map(() => '?').join(',');
    const deleteHistoryStmt = db.prepare(`
      DELETE FROM cook_history WHERE smoker_id IN (${placeholders})
    `);
    deleteHistoryStmt.run(...smokerIds);
    
    // Deregister the tenants (delete from smokers table)
    const deleteSmokersStmt = db.prepare(`
      DELETE FROM smokers WHERE id IN (${placeholders})
    `);
    deleteSmokersStmt.run(...smokerIds);
    
    console.log(`Purged history and deregistered ${offlineSmokers.length} offline smokers`);
  }
}

export function closeDatabase(): void {
  db.close();
}