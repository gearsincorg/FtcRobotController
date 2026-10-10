{
  "startPoint": {
    "x": 56,
    "y": 8,
    "headingDeg": 90,
    "locked": false
  },
  "lines": [
    {
      "id": "line-mv1sepvd-jhz0ru",
      "color": "#ffc516",
      "name": "Path 1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 56,
        "y": 36
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": 90,
        "endDeg": 180
      }
    },
    {
      "kind": "atomic",
      "id": "line-mv1sf0dk-w2zzbt",
      "endPoint": {
        "x": 21.464594127806553,
        "y": 68.09758203799655
      },
      "controlPoints": [
        {
          "x": 56.09499136442141,
          "y": 75.06476683937825
        }
      ],
      "heading": {
        "type": "constant",
        "reverse": true,
        "degrees": 180
      },
      "color": "#98BC78",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 118.3,
          "y": 141.5
        },
        {
          "x": 135.5,
          "y": 118
        },
        {
          "x": 136.3,
          "y": 70.2
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.2,
          "y": 116.9
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-mv1sepvd-jhz0ru"
    },
    {
      "kind": "path",
      "lineId": "line-mv1sf0dk-w2zzbt"
    }
  ],
  "fieldPoints": [],
  "activePaths": [],
  "settings": {
    "xVelocity": 63,
    "yVelocity": 34,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 15,
    "rHeight": 15,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 33,
    "maxDeceleration": 47,
    "fieldMap": "biobuzz.webp",
    "robotImage": "/robot.png",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false,
    "showHeadingArrow": true,
    "showCurrentTValue": true,
    "leftPanelWidth": 370,
    "rightPanelWidth": 620,
    "headingArrowLength": 55,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2.5,
    "pathOpacity": 1,
    "leftPanelMinWidth": 0,
    "rightPanelMinWidth": 0,
    "penToolMaxPaths": 8,
    "curveThroughMaxPoints": 4,
    "experimentalFeatures": {
      "optimize": true,
      "curveThrough": true,
      "obstacles": true
    }
  },
  "version": "1.5.0",
  "timestamp": "2026-10-10T02:41:34.375Z"
}