# senseBox sketch-templater
Arduino sketch templates used by the openSenseMap-api

[![Build Status](https://travis-ci.org/sensebox/node-sketch-templater.svg?branch=master)](https://travis-ci.org/sensebox/node-sketch-templater)
[![NPM Version](https://img.shields.io/npm/v/@sensebox/sketch-templater.svg)](https://www.npmjs.com/package/@sensebox/sketch-templater)

## Changelog

See [`CHANGELOG.md`](CHANGELOG.md)

#### Releasing a new version
On each commit, [Travis CI](https://travis-ci.org/sensebox/node-sketch-templater) builds and tests the package with this [`.travis.yml`](.travis.yml). If the commit has a Git tag, a new version of the package will be published to npm through travis.

To create a new version, use `npm version`.
1. Document your changes in [`CHANGELOG.md`](CHANGELOG.md). Make sure there are no uncommited changes in the worktree.
1. Run `npm version [major | minor | patch] -m "[v%s] Your commit message"`
1. Type in the new version (to create a `beta` release include the word `beta` in the new version)
1. `git push --tags origin master`

## Usage

Install via `npm install --save @sensebox/sketch-templater` or `yarn add @sensebox/sketch-templater`

```javascript

const Sketcher = require('@sensebox/sketch-templater');

const mySketcher = new Sketcher('<your api post domain>');

// generate Sketch
const mySketch = mySketcher.generateSketch(box);
const mySketchBase64 = mySketcher.generateSketch(box, { encoding: 'base64' });
```

### Configuration

In order to fill in the correct ingress domain, you have to specify a valid hostname. Do not specify a protocol (http or https)!

You can do this either in code when calling the `new Sketcher('your domain here')` or through external configuration.

If your project is using [`lorenwest/node-config`](https://github.com/lorenwest/node-config), you can specify the ingress domain in your config file of your project like this:

      {
        ... your other config

        "sketch-templater": {
          "ingress_domain": "ingress.example.com"
        }
      }

## Adding new Templates

To add new templates, just create a new `.tpl` file in the [`templates`](templates) directory. A template consists of two parts. The first line contains a JSON object for configuration. The second line until the end is used as the template text.

### Example configuration in first line

Specify a single model:
```json
{ "model": "homeEthernet" }
```

Specify multiple models:
```json
{ "models": ["homeWifi", "homeWifiFeinstaub"] }
```

### Templating values

The templater uses special transformers to process the templates. The transformers are applied through searching for `@@SUB_TEMPLATE_KEY@@` occurrences in the template files. Each replacement starts and ends with a double `@`. To specify a transformer, append a pipe (`|`) and the transformer name. When no transformer is specified, it just returns the input variable. For adding new template transformers, see [Adding Transformers](#adding-transformers).

As of writing this, the following replacements can be made:

| Template text | Replacement |
|------------------|-------------|
| `@@SENSEBOX_ID@@` | the senseBox ID  |
| `@@SENSOR_IDS@@` | sensor IDs |
| `@@NUM_SENSORS@@` | the number of sensors |
| `@@INGRESS_DOMAIN@@` | the domain of your ingress server |
| `@@SERIAL_PORT@@` | Serial port for connected SDS011 (only `Feinstaub` models) |
| `@@SSID@@` | the domain of your ingress server (only `homeV2Wifi` models) |
| `@@PASSWORD@@` | the domain of your ingress server (only `homeV2Wifi` models) |
| `@@POSTING_INTERVAL@@` | the posting interval in seconds (only `GPS` models) |

Additionally, the following transformers are implemented:

| Transformer name | Description |
|------------------|-------------|
| `as-is` | Do nothing. |
| `toDefine` | Transform an array of sensors to multiple `#define` statements. |
| `toProgmem` | Transform an array of sensors to multiple `const char xxSENSOR_ID[] PROGMEM = "<id>";` statements. |
| `digitalPortToPortNumber` | Transform a digital port (`A`, `B` or `C`) to a port number. You can add a offset parameter. |

## Adding Transformers

In order to add a new transformer, just add a function in [`src/transformers.js`](src/transformers.js) to the `module.exports`. The function should return a string.


## Adding Replacements

Add your additional replacements in [`src/index.js`](src/index.js) in the method `_cloneBox` to the second parameter of the `Object.assign` call.

## License

[MIT 2017 Matthias Pfeil, Jan Wirwahn, Gerald Pape](LICENSE)
