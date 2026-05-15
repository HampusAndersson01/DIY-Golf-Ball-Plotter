import type { ChangeEvent } from 'react'

type Props = {
  imagePreviewUrl: string | null
  hasAnalysis: boolean
  onFileChange: (event: ChangeEvent<HTMLInputElement>) => void
  onAnalyze: () => void
  disabled: boolean
}

export function ImageImportCard({ imagePreviewUrl, hasAnalysis, onFileChange, onAnalyze, disabled }: Props) {
  return (
    <section className="panel">
      <div className="panel-heading">
        <div>
          <div className="panel-kicker">Image</div>
          <h2>Import Image</h2>
        </div>
        <span className={`badge ${hasAnalysis ? 'good' : 'muted'}`}>{hasAnalysis ? 'Analyzed' : 'Awaiting file'}</span>
      </div>

      <label className="upload-zone">
        <input accept=".png,.jpg,.jpeg,image/png,image/jpeg" onChange={onFileChange} type="file" />
        <span>{imagePreviewUrl ? 'Replace image' : 'Select PNG/JPG'}</span>
      </label>

      <div className="thumb-frame">
        {imagePreviewUrl ? <img alt="Original upload" src={imagePreviewUrl} /> : <span>Image preview</span>}
      </div>

      <button className="button primary" disabled={disabled} onClick={onAnalyze} type="button">
        Analyze Colors
      </button>
    </section>
  )
}
