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
      smoker_temperature REAL,
      smoker_target REAL,
      cook_timer INTEGER,
      cook_time INTEGER,
      duty_cycle REAL,
      probe1_temperature REAL,
      probe1_target REAL,
      probe1_alarm INTEGER,
      probe2_temperature REAL,
      probe2_target REAL,
      probe2_alarm INTEGER,
      probe3_temperature REAL,
      probe3_target REAL,
      probe3_alarm INTEGER,
      probe4_temperature REAL,
      probe4_target REAL,
      probe4_alarm INTEGER,
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
      smoker_id, timestamp, smoker_temperature, smoker_target, cook_timer, cook_time, duty_cycle,
      probe1_temperature, probe1_target, probe1_alarm,
      probe2_temperature, probe2_target, probe2_alarm,
      probe3_temperature, probe3_target, probe3_alarm,
      probe4_temperature, probe4_target, probe4_alarm
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
  `);
  
  stmt.run(
    smokerId,
    timestamp,
    payload.smokerTemperature,
    payload.smokerTarget,
    payload.cookTimer,
    payload.cookTime ?? null,
    payload.dutyCycle ?? null,
    payload.probe1Temperature ?? null,
    payload.probe1Target ?? null,
    payload.probe1Alarm ? 1 : 0,
    payload.probe2Temperature ?? null,
    payload.probe2Target ?? null,
    payload.probe2Alarm ? 1 : 0,
    payload.probe3Temperature ?? null,
    payload.probe3Target ?? null,
    payload.probe3Alarm ? 1 : 0,
    payload.probe4Temperature ?? null,
    payload.probe4Target ?? null,
    payload.probe4Alarm ? 1 : 0 
  );
  
  // Update smoker status
  //updateSmokerStatus(smokerId, 'online');
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