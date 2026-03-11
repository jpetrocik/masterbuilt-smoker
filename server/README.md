# Server

A TypeScript Node.js API Server.

## Getting Started

1. Install dependencies:
   ```bash
   npm install
   ```

2. Create a `.env` file based on `.env.example`:
   ```bash
   cp .env.example .env
   ```

3. Run the development server:
   ```bash
   npm run dev
   ```

4. Build for production:
   ```bash
   npm run build
   npm start
   ```

## Project Structure

```
├── src/
│   ├── controllers/
│   ├── middleware/
│   ├── models/
│   ├── routes/
│   ├── types/
│   └── index.ts
├── tests/
├── .env.example
├── package.json
└── tsconfig.json
```

## API Endpoints

- `GET /` - Welcome message
- `GET /health` - Health check
- `GET /api` - API base route