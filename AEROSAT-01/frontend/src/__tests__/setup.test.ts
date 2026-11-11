// SET-04 (TEST_PLAN section 11.1): the frontend toolchain works as configured.
// No dashboard logic is involved.
import { expect, test } from "vitest";
import * as THREE from "three";
import tsconfig from "../../tsconfig.json";
import { App } from "../App";

test("SET-04 TypeScript strict mode is enabled (NFR-11)", () => {
  expect(tsconfig.compilerOptions.strict).toBe(true);
});

test("SET-04 the application shell module compiles and exports a component", () => {
  expect(typeof App).toBe("function");
});

test("SET-04 Three.js resolves and computes a known vector length", () => {
  expect(new THREE.Vector3(1, 2, 3).length()).toBeCloseTo(Math.sqrt(14), 12);
});
