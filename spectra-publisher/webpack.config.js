const path = require('path');

module.exports = {
    entry: './client_side.js',
    output: {
        path: path.resolve(__dirname, 'dist'),
        filename: 'bundle.js'
    }
};
