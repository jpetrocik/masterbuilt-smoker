import { DatabaseSync } from 'node:sqlite';
import path from 'path';
import config from '../config';
import { CookHistoryRow, SmokerTelemetryData } from '../types';

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
      lastSeen INTEGER NOT NULL
    )
  `);
  
// Create cook_history table
  db.exec(`
    CREATE TABLE IF NOT EXISTS cook_history (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      smokerId TEXT NOT NULL,
      timestamp INTEGER NOT NULL,
      smokerTemperature REAL,
      smokerTarget REAL,
      cookTimer INTEGER,
      cookTime INTEGER,
      dutyCycle REAL,
      probe1Temperature REAL,
      probe1Target REAL,
      probe1Alarm INTEGER,
      probe2Temperature REAL,
      probe2Target REAL,
      probe2Alarm INTEGER,
      probe3Temperature REAL,
      probe3Target REAL,
      probe3Alarm INTEGER,
      probe4Temperature REAL,
      probe4Target REAL,
      probe4Alarm INTEGER,
      FOREIGN KEY (smokerId) REFERENCES smokers(id) ON DELETE CASCADE
    )
  `);
  
  // Create fcm_tokens table
  db.exec(`
    CREATE TABLE IF NOT EXISTS fcm_tokens (
      id INTEGER PRIMARY KEY AUTOINCREMENT,
      smokerId TEXT NOT NULL,
      token TEXT NOT NULL UNIQUE,
      updated_at INTEGER NOT NULL,
      FOREIGN KEY (smokerId) REFERENCES smokers(id) ON DELETE CASCADE
    )
  `);
  
  // Create index for faster queries
  db.exec(`
    CREATE INDEX IF NOT EXISTS idx_cook_history_smoker_timestamp 
    ON cook_history(smokerId, timestamp)
  `);
}

export function registerFcmToken(smokerId: string, token: string): void {
  const stmt = db.prepare(`
    INSERT OR REPLACE INTO fcm_tokens (smokerId, token, updated_at)
    VALUES (?, ?, ?)
  `);
  stmt.run(smokerId, token, Date.now());
}

export function getTokensForSmoker(smokerId: string): string[] {
  const stmt = db.prepare('SELECT token FROM fcm_tokens WHERE smokerId = ?');
  const results = stmt.all(smokerId) as { token: string }[];
  return results.map(row => row.token);
}

export function insertTelemetry(data: SmokerTelemetryData): void {
  
  // Insert into cook_history
  const stmt = db.prepare(`
    INSERT INTO cook_history (
      smokerId, timestamp, smokerTemperature, smokerTarget, cookTimer, cookTime, dutyCycle,
      probe1Temperature, probe1Target, probe1Alarm,
      probe2Temperature, probe2Target, probe2Alarm,
      probe3Temperature, probe3Target, probe3Alarm,
      probe4Temperature, probe4Target, probe4Alarm
    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
  `);
  
  stmt.run(
    data.smokerId,
    data.timestamp,
    data.smokerTemperature,
    data.smokerTarget,
    data.cookTimer,
    data.cookTime ?? null,
    data.dutyCycle ?? null,
    data.probe1Temperature ?? null,
    data.probe1Target ?? null,
    data.probe1Alarm ? 1 : 0,
    data.probe2Temperature ?? null,
    data.probe2Target ?? null,
    data.probe2Alarm ? 1 : 0,
    data.probe3Temperature ?? null,
    data.probe3Target ?? null,
    data.probe3Alarm ? 1 : 0,
    data.probe4Temperature ?? null,
    data.probe4Target ?? null,
    data.probe4Alarm ? 1 : 0 
  );
  
  // Update smoker status
  //updateSmokerStatus(smokerId, 'online');
}

export function updateSmokerStatus(smokerId: string, status: 'online' | 'offline'): void {
  const timestamp = Date.now();
  
  const stmt = db.prepare(`
    INSERT OR REPLACE INTO smokers (id, status, lastSeen)
    VALUES (?, ?, ?)
  `);
  
  stmt.run(smokerId, status, timestamp);
}

export function getSmokers(filterStatus?: 'online' | 'offline' | 'all'): { id: string; status: string; lastSeen: number }[] {
  let sql = 'SELECT id, status, lastSeen FROM smokers';
  
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
    WHERE smokerId = ? AND timestamp >= ? 
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
  // First, get smokers that are offline and lastSeen < offlineCutoff
  const offlineSmokersStmt = db.prepare(`
    SELECT id FROM smokers 
    WHERE status = 'offline' AND lastSeen < ?
  `);
  const offlineSmokers = offlineSmokersStmt.all(offlineCutoff) as { id: string }[];
  
  if (offlineSmokers.length > 0) {
    const smokerIds = offlineSmokers.map(s => s.id);
    
    // Delete history for these smokers
    const placeholders = smokerIds.map(() => '?').join(',');
    const deleteHistoryStmt = db.prepare(`
      DELETE FROM cook_history WHERE smokerId IN (${placeholders})
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
