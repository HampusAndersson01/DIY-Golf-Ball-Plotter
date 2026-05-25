---
name: Industrial Precision
colors:
  surface: '#f8f9ff'
  surface-dim: '#cbdbf5'
  surface-bright: '#f8f9ff'
  surface-container-lowest: '#ffffff'
  surface-container-low: '#eff4ff'
  surface-container: '#e5eeff'
  surface-container-high: '#dce9ff'
  surface-container-highest: '#d3e4fe'
  on-surface: '#0b1c30'
  on-surface-variant: '#434655'
  inverse-surface: '#213145'
  inverse-on-surface: '#eaf1ff'
  outline: '#737686'
  outline-variant: '#c3c6d7'
  surface-tint: '#0053db'
  primary: '#004ac6'
  on-primary: '#ffffff'
  primary-container: '#2563eb'
  on-primary-container: '#eeefff'
  inverse-primary: '#b4c5ff'
  secondary: '#006c49'
  on-secondary: '#ffffff'
  secondary-container: '#6cf8bb'
  on-secondary-container: '#00714d'
  tertiary: '#784b00'
  on-tertiary: '#ffffff'
  tertiary-container: '#996100'
  on-tertiary-container: '#ffeedd'
  error: '#ba1a1a'
  on-error: '#ffffff'
  error-container: '#ffdad6'
  on-error-container: '#93000a'
  primary-fixed: '#dbe1ff'
  primary-fixed-dim: '#b4c5ff'
  on-primary-fixed: '#00174b'
  on-primary-fixed-variant: '#003ea8'
  secondary-fixed: '#6ffbbe'
  secondary-fixed-dim: '#4edea3'
  on-secondary-fixed: '#002113'
  on-secondary-fixed-variant: '#005236'
  tertiary-fixed: '#ffddb8'
  tertiary-fixed-dim: '#ffb95f'
  on-tertiary-fixed: '#2a1700'
  on-tertiary-fixed-variant: '#653e00'
  background: '#f8f9ff'
  on-background: '#0b1c30'
  surface-variant: '#d3e4fe'
typography:
  headline-lg:
    fontFamily: Inter
    fontSize: 32px
    fontWeight: '700'
    lineHeight: 40px
    letterSpacing: -0.02em
  headline-md:
    fontFamily: Inter
    fontSize: 24px
    fontWeight: '600'
    lineHeight: 32px
    letterSpacing: -0.01em
  headline-sm:
    fontFamily: Inter
    fontSize: 20px
    fontWeight: '600'
    lineHeight: 28px
  body-lg:
    fontFamily: Inter
    fontSize: 16px
    fontWeight: '400'
    lineHeight: 24px
  body-md:
    fontFamily: Inter
    fontSize: 14px
    fontWeight: '400'
    lineHeight: 20px
  label-md:
    fontFamily: JetBrains Mono
    fontSize: 13px
    fontWeight: '500'
    lineHeight: 16px
    letterSpacing: 0.02em
  label-sm:
    fontFamily: JetBrains Mono
    fontSize: 11px
    fontWeight: '500'
    lineHeight: 14px
    letterSpacing: 0.05em
  headline-lg-mobile:
    fontFamily: Inter
    fontSize: 24px
    fontWeight: '700'
    lineHeight: 32px
rounded:
  sm: 0.25rem
  DEFAULT: 0.5rem
  md: 0.75rem
  lg: 1rem
  xl: 1.5rem
  full: 9999px
spacing:
  base: 8px
  xs: 4px
  sm: 12px
  md: 24px
  lg: 40px
  xl: 64px
  gutter: 24px
  margin: 32px
---

## Brand & Style

This design system is engineered for professional industrial environments, maker-labs, and CAM/CNC control interfaces. The brand personality is grounded in **reliability, precision, and efficiency**. It avoids decorative flourishes in favor of high-utility layouts that minimize cognitive load during complex operations.

The visual style is **Corporate Modern with a Technical Edge**. It utilizes generous whitespace to prevent data fatigue, while maintaining a structured grid that feels robust and engineered. The emotional response is one of calm control—providing the user with the confidence that the software is as precise as the hardware it commands.

## Colors

The palette is anchored by a professional **Action Blue** for primary interactions and navigational elements. Status colors are utilized with high-contrast intent: **Ready Green** for successful states, **Calibration Amber** for warnings, and **Emergency Red** for critical failures or stop actions.

The neutral scale favors cool-toned slates to provide a clean, "laboratory" feel. Backgrounds use subtle gray-to-white transitions to create a sense of depth and hierarchy without relying on heavy lines.

## Typography

This design system uses **Inter** for all primary UI text to ensure maximum legibility and a professional, neutral tone. To support the "maker-lab" aesthetic and provide clear differentiation for technical metrics (coordinates, RPM, temperature, feed rates), **JetBrains Mono** is utilized for labels and numerical data.

Hierarchy is strictly maintained through weight and scale. Headlines are bold and slightly condensed to feel impactful, while body text remains airy and readable for documentation and log files.

## Layout & Spacing

The layout follows a **12-column fluid grid** for the main dashboard content, allowing modules to resize based on screen real estate. A standard 8px base unit (the "module") governs all spacing decisions to ensure mathematical consistency.

- **Desktop:** 24px gutters with 32px outer margins. Content blocks typically span 3, 4, 6, or 12 columns.
- **Tablet:** 16px gutters and margins; sidebars often collapse into icons.
- **Mobile:** Single column stack with 16px margins to maximize touch targets and readability of telemetry data.

## Elevation & Depth

Visual hierarchy is established through **Tonal Layers** and **Ambient Shadows**. Surfaces are kept white or very light gray to maintain a high-contrast environment.

- **Level 0 (Background):** Soft gray (#f8fafc), non-interactive.
- **Level 1 (Cards/Panels):** Pure white with a 1px border (#e2e8f0) and a very soft, diffused shadow (0px 4px 6px rgba(0,0,0,0.05)).
- **Level 2 (Dropdowns/Modals):** Increased shadow depth (0px 10px 15px rgba(0,0,0,0.1)) to indicate focus and separation from the grid.

Floating Action Buttons (FABs) or primary control triggers use a subtle inner-shadow to give a tactile, "pressable" appearance without appearing skeuomorphic.

## Shapes

The design system employs a **Rounded** shape language. A standard corner radius of 8px (0.5rem) is used for all primary UI components like buttons and cards. This softens the industrial nature of the data while maintaining a clean, modern silhouette.

Buttons utilize the standard 8px radius, while larger container elements (like dashboard widgets) may scale up to 16px (rounded-lg) to create a clear visual nesting effect. Success/Warning indicators often use pill-shaped (rounded-full) styling for maximum visibility and differentiation from square data cells.

## Components

- **Buttons:** Solid fills for primary actions using the Action Blue. Secondary buttons use a ghost style (border only). Emergency buttons use a heavy Danger Red fill with bold white typography.
- **Chips & Status:** High-contrast pills. Ready states use a subtle green tint background with dark green text. Warnings use amber backgrounds with high-contrast icon indicators.
- **Input Fields:** 1px slate borders that thicken and change to Action Blue on focus. Labels are always positioned above the field using JetBrains Mono for a technical feel.
- **Cards:** Clean white containers with 8px radius. Titles are separated by a subtle 1px divider or a light gray header area.
- **Data Tables:** Dense but legible. Row hovering uses a very light blue tint. Numerical columns use JetBrains Mono for perfect vertical alignment of decimal points.
- **Gauges & Metrics:** Minimalist circular or linear progress bars using the primary action color. Avoid gradients; use solid color blocks to indicate threshold breaches.