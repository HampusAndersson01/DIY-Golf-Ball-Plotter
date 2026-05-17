export function parseLocaleNumber(value: string): number {
  return Number(String(value).replace(',', '.'))
}
