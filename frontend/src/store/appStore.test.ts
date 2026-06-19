import { beforeEach, describe, expect, it } from 'vitest'

import type { ImageAnalysis } from '../api/types'
import { useAppStore } from './appStore'

const singleColorAnalysis: ImageAnalysis = {
  width: 16,
  height: 16,
  colors: [
    {
      id: 'color-group-0',
      hex: '#000000',
      rgb: [0, 0, 0],
      pixel_count: 256,
      coverage: 1,
      coverage_percent: 100,
      luminance: 0,
      is_transparent: false,
    },
  ],
}

describe('appStore color selection', () => {
  beforeEach(() => {
    useAppStore.setState({
      analysis: null,
      selectedColors: [],
    })
  })

  it('auto-selects the only detected printable color', () => {
    useAppStore.getState().setAnalysis(singleColorAnalysis)
    expect(useAppStore.getState().selectedColors).toEqual(['color-group-0'])
  })

  it('clears auto-selection when multiple printable colors are detected', () => {
    useAppStore.getState().setAnalysis({
      ...singleColorAnalysis,
      colors: [
        singleColorAnalysis.colors[0],
        {
          ...singleColorAnalysis.colors[0],
          id: 'color-group-1',
          hex: '#FF0000',
          rgb: [255, 0, 0],
        },
      ],
    })
    expect(useAppStore.getState().selectedColors).toEqual([])
  })
})
