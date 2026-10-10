{
  "startPoint": {
    "x": 56,
    "y": 8,
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-mv1cez0q-fzirz3",
      "color": "#ffc516",
      "name": "Path 1",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 57.955094991364426,
        "y": 27.202072538860104
      },
      "controlPoints": [],
      "heading": {
        "type": "linear",
        "startDeg": 90,
        "endDeg": 90
      }
    },
    {
      "kind": "atomic",
      "id": "line-mv1r446v-7pwjhj",
      "endPoint": {
        "x": 58.566493955094984,
        "y": 75.3091537132988
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": false
      },
      "color": "#D59575",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mv1r47vi-t8f0b7",
      "endPoint": {
        "x": 118.59326424870468,
        "y": 94.32124352331604
      },
      "controlPoints": [
        {
          "x": 58.676165803108816,
          "y": 119.18652849740936
        },
        {
          "x": 104.14162348877375,
          "y": 94.93523316062179
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false
      },
      "color": "#9DB97A",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mv1r4ca6-7xcaa6",
      "endPoint": {
        "x": 128.78238341968913,
        "y": 94.00259067357513
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": false
      },
      "color": "#798A7B",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "kind": "atomic",
      "id": "line-mv1rma9g-885dei",
      "endPoint": {
        "x": 119.0993091537133,
        "y": 94.16407599309153
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": true
      },
      "color": "#C69AC7",
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
      "lineId": "line-mv1cez0q-fzirz3"
    },
    {
      "kind": "path",
      "lineId": "line-mv1r446v-7pwjhj"
    },
    {
      "kind": "path",
      "lineId": "line-mv1r47vi-t8f0b7"
    },
    {
      "kind": "path",
      "lineId": "line-mv1r4ca6-7xcaa6"
    },
    {
      "kind": "path",
      "lineId": "line-mv1rma9g-885dei"
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
  "timestamp": "2026-10-10T02:18:59.042Z"
}