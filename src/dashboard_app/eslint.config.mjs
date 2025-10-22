import { FlatCompat } from "@eslint/eslintrc";
import eslintPluginBetterTailwindcss from "eslint-plugin-better-tailwindcss";

const compat = new FlatCompat({
  baseDirectory: import.meta.dirname,
});

const eslintConfig = [
  {
    ignores: [
      "node_modules/**",
      ".next/**",
      "out/**",
      "build/**",
      "next-env.d.ts",
    ],
  },
  ...compat.extends("next/core-web-vitals", "next/typescript"),
  ...compat.config({
    extends: ["next", "next/typescript", "next/core-web-vitals"],
    plugins: ["@typescript-eslint", "better-tailwindcss"],
    rules: {
      // enable all recommended rules to report a warning
      ...eslintPluginBetterTailwindcss.configs["recommended-warn"].rules,
      // enable all recommended rules to report an error
      ...eslintPluginBetterTailwindcss.configs["recommended-error"].rules,

      "better-tailwindcss/enforce-consistent-line-wrapping": [
        "warn",
        { printWidth: 80 },
      ],
    },
  }),
  {
    settings: {
      "better-tailwindcss": {
        entryPoint: "src/app/globals.css",
        callees: ["cn", "classnames", "clsx", "ctl", "cva"],
      },
    },
  },
];

export default eslintConfig;
