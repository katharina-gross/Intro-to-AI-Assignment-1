const express = require('express');
const { exec } = require('child_process');
const path = require('path');
const fs = require('fs');

const app = express();
const PORT = 3000;

// Middleware
app.use(express.static('.'));
app.use(express.json());

// Routes
app.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, 'index.html'));
});

app.post('/api/run-test', (req, res) => {
    const { testCount = 1 } = req.body;

    exec(`./pathfinder --batch-test ${testCount}`, (error, stdout, stderr) => {
        if (error) {
            return res.status(500).json({ error: error.message });
        }

        // Read test results
        try {
            const summary = JSON.parse(fs.readFileSync('test_results/summary.json', 'utf8'));
            res.json(summary);
        } catch (e) {
            res.status(500).json({ error: 'Failed to read test results' });
        }
    });
});

app.get('/api/test/:id', (req, res) => {
    const testId = req.params.id;
    const testPath = path.join(__dirname, 'test_results', `test_${testId}.json`);

    if (fs.existsSync(testPath)) {
        const testData = JSON.parse(fs.readFileSync(testPath, 'utf8'));
        res.json(testData);
    } else {
        res.status(404).json({ error: 'Test not found' });
    }
});

app.get('/api/test-results', (req, res) => {
    const summaryPath = path.join(__dirname, 'test_results', 'summary.json');

    if (fs.existsSync(summaryPath)) {
        const summary = JSON.parse(fs.readFileSync(summaryPath, 'utf8'));
        res.json(summary);
    } else {
        res.status(404).json({ error: 'No test results found' });
    }
});

app.listen(PORT, () => {
    console.log(`🚀 Server running at http://localhost:${PORT}`);
    console.log(`📊 Pathfinding Tester Dashboard is ready!`);
});