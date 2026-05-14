const foxglove = require("@foxglove/eslint-plugin");

module.exports = [
  ...foxglove.configs.base,
  ...foxglove.configs.react,
  {
    ignores: ["dist/**"],
  },
];