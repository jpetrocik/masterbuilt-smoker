// server/src/firebase/firebaseAdmin.ts
import * as admin from 'firebase-admin';
import * as dotenv from 'dotenv';

dotenv.config();

const serviceAccountPath = process.env.FIREBASE_SERVICE_ACCOUNT_PATH;

if (!serviceAccountPath) {
  throw new Error('FIREBASE_SERVICE_ACCOUNT_PATH is not set in the environment variables.');
}

admin.initializeApp({
  credential: admin.credential.cert(serviceAccountPath),
});

export const messaging = admin.messaging();
