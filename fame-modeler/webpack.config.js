const CopyWebpackPlugin = require('copy-webpack-plugin');
const { DefinePlugin } = require('webpack');

const path = require('path');

module.exports = {
  mode: 'development',
  entry: {
    viewer: './fame-modeler/src/viewer.js',
    modeler: './fame-modeler/src/modeler.js'
  },
  output: {
    path: path.resolve(__dirname, 'fame-modeler'),
    filename: '[name].js'
  },
  devtool: 'source-map',
  module: {
    rules: [
      {
        test: /\.less$/,
        use: [
          'style-loader',
          'css-loader',
          'less-loader',
        ],
      },
      {
        test: /\.css$/,
        use: [
          'style-loader',
          'css-loader',
        ]
      },
      {
        test: /\.bpmn$/,
        use: {
          loader: 'raw-loader'
        }
      }
    ]
  },
  plugins: [
    new CopyWebpackPlugin({
      patterns: [
        { from: 'fame-modeler/index.html', to: '.' }
      ]
    }),
    new DefinePlugin({
      'process.env.TOKEN_SIMULATION_VERSION': JSON.stringify(require('./package.json').version)
    })
  ]
};